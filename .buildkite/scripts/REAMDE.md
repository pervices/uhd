# Additional Scripts to Fix Permissions due to Docker when using Buildkite

For more info, see [this](https://buildkite.com/docs/agent/v3/docker#permissions-errors-when-using-docker).

Originally borrowed from [here](https://github.com/buildkite/elastic-ci-stack-for-aws/blob/v2.3.4/packer/conf/buildkite-agent).

# sudoers.conf:

Add the two lines below into /etc/sudoers using the command `sudo visudo`. I've tried adding them in a separate file under
/etc/sudoers.d/ but that does not work for some reason.

These entries give permissions for the `buildkite-agent` user to call the scripts below without a password. They scripts are
given root privileges and therefore, some caution should be taken inside of the scripts themselves.

The scripts below should be installed at `/usr/bin` on the machine where `buildkite-agent` is installed. They should be called
within the following hooks:

* `/etc/buildkite-agent/hooks/environment`

## fix-buildkite-agent-ccache-permissions

The purpose of this script is to recursively fix permissions of the buildkite ccache directory after the contents have possibly had
ownership changes corresponding to the Docker default `user:group`, `0:0`, which maps to `root:root` on most systems.

The script should be called as follows.

```bash
sudo /usr/bin/fix-buildkite-agent-ccache-permissions
```

## fix-buildkite-agent-builds-permissions

The purpose of this script is to recursively fix permissions of the buildkite builds directory after the contents have possibly had
ownership changes corresponding to the Docker default `user:group`, `0:0`, which maps to `root:root` on most systems.

The script should be called as follows.

```bash
sudo /usr/bin/fix-buildkite-agent-builds-permissions \
	"${BUILDKITE_AGENT_NAME}" \
	"${BUILDKITE_ORGANIZATION_SLUG}" \
	"${BUILDKITE_PIPELINE_NAME}"
```

If the directory below exists, it will have ownership recursively changed to `buildkite-agent:buildkite-agent`.

```bash
/var/lib/buildkite-agent/builds/${BUILDKITE_AGENT_NAME}/${BUILDKITE_ORGANIZATION_SLUG}/${BUILDKITE_PIPELINE_NAME}
```
