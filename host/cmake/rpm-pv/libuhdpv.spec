%ifarch aarch64
%bcond_without neon
%else
%bcond_with neon
%endif
%undefine _annotated_build
%undefine _hardened_build
%ifarch %{arm}
%if %{with neon}
%global my_optflags %(echo -n "%{optflags}" | sed 's/-mfpu=[^ \\t]\\+//g'; echo " -mfpu=neon")
%{expand: %global optflags %{my_optflags}}
%global mfpu_neon -Dhave_mfpu_neon=1
%else
%global mfpu_neon -Dhave_mfpu_neon=0
%endif
%endif
%global __python3 /usr/bin/python3.11
%global python3_pkgversion 3.11

#Disable generation of the debug packages because as we haven't quite figured out
#a way to package them that doesn't also cause rpm build issues.
%global debug_package %{nil}
%global real_name uhd

Name:           libuhdpv
URL:            http://github.com/pervices/uhd
Version:        libuhdpv_ver
Release:        master
License:        GPLv3+
Provides: libuhdpv
Conflicts: uhd, libuhd
BuildRequires:  gcc-toolset-13
BuildRequires:  cmake, git
BuildRequires:  boost169-python3-devel, libusb1-devel, python%{python3_pkgversion}-cheetah, ncurses-devel
BuildRequires:  python%{python3_pkgversion}-docutils, doxygen, pkgconfig, libpcap-devel
BuildRequires:  python%{python3_pkgversion}-numpy, vim-common, octave, dpdk
BuildRequires:  python%{python3_pkgversion}, python%{python3_pkgversion}-setuptools
%if %{with wireshark}
BuildRequires:  wireshark-devel
%endif
BuildRequires:  python%{python3_pkgversion}-mako, python%{python3_pkgversion}-requests, python%{python3_pkgversion}-devel, tar
%if ! %{with binary_firmware}
BuildRequires: sed
%endif
Requires(pre):  shadow-utils, glibc-common
Requires:  python%{python3_pkgversion}-numpy
Summary:        Universal Hardware Driver for Ettus Research products
#Source0:       %%{url}/archive/v%%{version}/uhd-%%{version}.tar.gz
Source0:        uhdpv.tar.gz

%description
The UHD is the universal hardware driver for Ettus Research products.
The goal of the UHD is to provide a host driver and API for current and
future Ettus Research products. It can be used standalone without GNU Radio.


%prep
%setup -q -n %{real_name}


%build
source /opt/rh/gcc-toolset-13/enable
# fix python shebangs (run again for generated scripts)
find . -type f -name "*.py" -exec sed -i '/^#!/ s|.*|#!%{__python3}|' {} \;
mkdir -p host/build
pushd host/build
%cmake %{?have_neon} -DPKG_LIB_DIR="/usr/lib/uhd" -DCMAKE_INSTALL_PREFIX="/usr" -DENABLE_CRIMSON_TNG="ON" -DENABLE_EXAMPLES="ON" -DENABLE_TESTS="OFF"  -DENABLE_N300="OFF"  -DENABLE_E320="OFF" -DENABLE_USRP1="OFF" -DENABLE_B200="OFF" -DENABLE_X300="OFF" -DENABLE_OCTOCLOCK="OFF" -DENABLE_DOXYGEN="OFF" -DENABLE_LIBURING="OFF" -DENABLE_PYTHON_API="ON" \
 ../
make %{?_smp_mflags}
#make -j1 
popd

# tools
#pushd tools/uhd_dump
#make %%{?_smp_mflags} CFLAGS="%%{optflags}" LDFLAGS="%%{?__global_ldflags}"
#popd


#%%check
#cd host/build
#make test

%install
pushd host/build
make install DESTDIR=%{buildroot}

popd
# Package base docs to base package
mkdir _tmpdoc
mv %{buildroot}%{_docdir}/uhd/{LICENSE,README.md} _tmpdoc


# remove win stuff
rm -rf %{buildroot}%{_datadir}/uhd/images/winusb_driver

# convert hardlinks to symlinks (to not package the file twice)
pushd %{buildroot}%{_bindir}
for f in uhd_images_downloader usrp2_card_burner
do
  unlink $f
  ln -s ../..%{_libexecdir}/uhd/${f}.py $f
done
popd

# tools
install -Dpm 0755 tools/usrp_x3xx_fpga_jtag_programmer.sh %{buildroot}%{_bindir}/usrp_x3xx_fpga_jtag_programmer.sh
#install -Dpm 0755 tools/uhd_dump/chdr_log %%{buildroot}%%{_bindir}/chdr_log

%if %{with wireshark}
# wireshark dissectors
pushd tools/dissectors
for d in %{wireshark_dissectors}
do
  pushd "build_$d"
  %make_install
  popd
done
popd
mv %{buildroot}${HOME}/.wireshark %{buildroot}%{_libdir}/wireshark
%endif

%ldconfig_scriptlets

%pre
getent group usrp >/dev/null || \
  %{_sbindir}/groupadd -r usrp >/dev/null 2>&1
exit 0

%post
sysctl net.core.rmem_max=500000000
sysctl net.core.wmem_max=500000000
exit 0

%files
%doc _tmpdoc/*
%doc %{_docdir}/uhd/doxygen
%{_includedir}/*
%{_bindir}/*
%{_mandir}/man1/*.1*
%{_datadir}/uhd
%{python3_sitearch}/uhd
%{python3_sitearch}/usrp_mpm/*
# %%exclude /usr/lib/debug/*
/usr/lib/*
/usr/lib64/libuhd.*
/usr/lib64/cmake/*
/usr/lib64/pkgconfig/uhd.pc

%changelog
