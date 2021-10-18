pipeline {
	agent none
	triggers {
		pollSCM('TZ=America/Toronto\nH/10 * * * *')
		//Polls every 10 minutes
	}
	options {
		ansiColor('xterm')
	}
	stages {
		stage('build') {
			agent {
				label 'uhd'
			}
			steps {
				sh 'echo This is just a proxy'
			}
		}
	}
	post {
		always {
			echo 'The build is finished, cleaning up workspace...'
			//might need to use deleteDir() to clean up workspace
		}
		failure {
			mail to: 'tech.f@pervices.com',
			subject: "Failed Pipeline: ${currentBuild.fullDisplayName}",
			body: "Something is wrong with the build ${env.BUILD_URL}"
		}
	}
}
