pipeline {
  agent {
    // docker { image 'ros:kinetic-ros-base-xenial' }
    docker { image 'joinaero/kinetic-ros-opencv-xenial' }
  }

  /*
  environment {
    // FindOpenCV.cmake
    OpenCV_DIR = '/opt/ros/kinetic/share/OpenCV-3.3.1-dev'
  }
  */

  stages {
    stage('Prepare') {
      steps {
        echo "WORKSPACE: ${env.WORKSPACE}"
        echo 'apt-get ..'
        sh '''
			apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
			apt-get update
		'''
      }
    }
    stage('Init') {
      steps {
        echo 'make init ..'
        sh 'make init INIT_OPTIONS=-y'
        // echo 'skip get submodules and make test'
        // sh './scripts/init.sh -y'
      }
    }
    stage('Build') {
      steps {
        echo 'make build ..'
        sh '. /opt/ros/kinetic/setup.sh; make build'
      }
    }
    stage('Install') {
      steps {
        echo 'make install ..'
        sh '. /opt/ros/kinetic/setup.sh; make install SUDO='
      }
    }

    stage('Samples') {
      steps {
        echo 'make samples ..'
        sh '. /opt/ros/kinetic/setup.sh; make samples SUDO='
      }
    }
	   /*
    stage('Tools') {
      steps {
        echo 'make tools ..'
        sh '. /opt/ros/kinetic/setup.sh; make tools SUDO='
      }
    }
	    */
    stage('ROS') {
      steps {
        echo 'make ros ..'
        sh '''
        . /opt/ros/kinetic/setup.sh
        rosdep install --from-paths wrappers/ros/src --ignore-src --rosdistro kinetic -y
        make ros SUDO=
        '''
      }
    }
    /*
    stage('Clean') {
      steps {
        echo 'clean ..'
        sh '''
        rm -rf /var/lib/apt/lists/*
        '''
      }
    }
    */
  }

  post {
    always {
      echo 'This will always run'
	  /*
	  dingTalk accessToken: '7dca6ae9b1b159b8b4b375e858b71f2e6cec8f73fa20d07552d09791261b2344',
                    imageUrl: 'http://icon-park.com/imagefiles/loading7_gray.gif',
                    message: '开始构建',
                    jenkinsUrl: "${JENKINS_URL}"
					*/

    }
    success {
      echo 'This will run only if successful'
	  /*
	  dingTalk accessToken: '7dca6ae9b1b159b8b4b375e858b71f2e6cec8f73fa20d07552d09791261b2344',
                    imageUrl: 'http://icons.iconarchive.com/icons/paomedia/small-n-flat/1024/sign-check-icon.png',
                    message: '构建成功',
                    jenkinsUrl: "${JENKINS_URL}"
*/
    }
    failure {
      echo 'This will run only if failed'
	  dingTalk accessToken: '7dca6ae9b1b159b8b4b375e858b71f2e6cec8f73fa20d07552d09791261b2344',
                    imageUrl: 'http://www.iconsdb.com/icons/preview/soylent-red/x-mark-3-xxl.png',
                    message: '构建失败',
                    jenkinsUrl: "${JENKINS_URL}"
    }
    unstable {
      echo 'This will run only if the run was marked as unstable'
    }
    changed {
      echo 'This will run only if the state of the Pipeline has changed'
      echo 'For example, if the Pipeline was previously failing but is now successful11'
    }
  }
}
