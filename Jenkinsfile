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
      mail to: 'mynteye-ci@slightech.com',
      subject: "编译失败 Failed Pipeline: ${currentBuild.fullDisplayName}",
      body: '''<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
</head>
<body leftmargin="8" marginwidth="0" topmargin="8" marginheight="4"
    offset="0">
    <table width="95%" cellpadding="0" cellspacing="0"
        style="font-size: 11pt; font-family: Tahoma, Arial, Helvetica, sans-serif">
        <tr>
            <td><br />
            <b><font color="#0B610B">构建信息</font></b>
            <hr size="2" width="100%" align="center" /></td>
        </tr>
        <tr>
            <td>
                <ul> 
                    <li>构建名称：${JOB_NAME}</li>
                    <li>构建结果: <span style="color:red"> ${BUILD_STATUS}</span></li>  
                    <li>构建编号：${BUILD_NUMBER}  </li>
                    <li>GIT 地址：${git_url}</li>                    
                    <li>GIT 分支：${git_branch}</li>
                    <li>变更记录: ${CHANGES,showPaths=true,showDependencies=true,format="<pre><ul><li>提交ID: %r</li><li>提交人：%a</li><li>提交时间：%d</li><li>提交信息：%m</li><li>提交文件：%p</li></ul></pre>",pathFormat="%p <br />"}
                </ul>
            </td>
        </tr>
        <tr>
            <td><b><font color="#0B610B">构建日志 :</font></b>
            <hr size="2" width="100%" align="center" /></td>
        </tr>
        <tr>
            <td><textarea cols="150" rows="30" readonly="readonly"
                    style="font-family: Courier New">${BUILD_LOG}</textarea>
            </td>
        </tr>
    </table>
</body>
</html>
'''
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
