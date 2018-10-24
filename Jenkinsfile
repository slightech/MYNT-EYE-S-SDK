pipeline {
  agent {
    docker { image 'ros:kinetic-ros-core-xenial' }
  }

  stages {
    stage('Prepare') {
      steps {
				echo "WORKSPACE: ${env.WORKSPACE}"
        echo 'apt-get ..'
        sh '''
        apt-get update
        apt-get install -y ros-kinetic-opencv3
        export OpenCV_DIR=/opt/ros/kinetic/share/OpenCV-3.3.1-dev
        '''
      }
    }
    stage('Init') {
      steps {
        echo 'make init ..'
        sh 'make init INIT_OPTIONS=-y'
      }
    }
    stage('Build') {
      steps {
        echo 'make build ..'
        sh 'make build'
      }
    }
    stage('Install') {
      steps {
        echo 'make install ..'
        sh 'make install SUDO='
      }
    }
    stage('Test') {
      steps {
        echo 'make test ..'
        sh 'make test SUDO='
      }
    }
    stage('Samples') {
      steps {
        echo 'make samples ..'
        sh 'make samples SUDO='
      }
    }
    stage('Tools') {
      steps {
        echo 'make tools ..'
        sh 'make tools SUDO='
      }
    }
    stage('ROS') {
      steps {
        echo 'make ros ..'
        sh '''
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
    }
    success {
      echo 'This will run only if successful'
    }
    failure {
      echo 'This will run only if failed'
      mail to: 'mynteye@slightech.com',
      subject: "Failed Pipeline: ${currentBuild.fullDisplayName}",
      body: "Something is wrong with ${env.BUILD_URL}"
    }
    unstable {
      echo 'This will run only if the run was marked as unstable'
    }
    changed {
      echo 'This will run only if the state of the Pipeline has changed'
      echo 'For example, if the Pipeline was previously failing but is now successful'
    }
  }
}
