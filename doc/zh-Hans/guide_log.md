# 日志 {#guide_log}

日志系统用的 `glog` ，通用配置在头文件 `glog_init.h` 里。

* 日志文件会存储在当前工作目录， `make cleanlog` 可以清理。
* 如果需要打开详细日志，请取消 `glog_init.h` 里注释的 `FLAGS_v = 2;` ，重新编译。
