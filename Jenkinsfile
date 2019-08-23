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
        sh 'apt-get update'
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
    stage('Test') {
      steps {
        echo 'make test ..'
        sh '. /opt/ros/kinetic/setup.sh; make test SUDO='
      }
    }
    stage('Samples') {
      steps {
        echo 'make samples ..'
        sh '. /opt/ros/kinetic/setup.sh; make samples SUDO='
      }
    }
    stage('Tools') {
      steps {
        echo 'make tools ..'
        sh '. /opt/ros/kinetic/setup.sh; make tools SUDO='
      }
    }
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
    }
    success {
      echo 'This will run only if successful'
    }
    failure {
      echo 'This will run only if failed'
	  // mail to: 'mynteye-ci@slightech.com',
      mail to: 'cocowang@slightech.com',
      subject: "${env.JOB_NAME} 编译失败 Failed Pipeline: ${currentBuild.fullDisplayName}",
      body: """
                详情：
                FAILED       : Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'             
                状态         ：${env.JOB_NAME} jenkins 运行失败 
                URL          ：${env.BUILD_URL}
                项目名称     ：${env.JOB_NAME} 
                项目更新进度 ：${env.BUILD_NUMBER}
            """
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
