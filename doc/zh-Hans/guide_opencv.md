# OpenCV 依赖 {#guide_opencv}

SDK 提供了三层接口，其 OpenCV 依赖情况如下：

* `api`， 上层接口，依赖 OpenCV 。
* `device`，中间层接口，不依赖 OpenCV 。
* `uvc`，底层接口，不依赖 OpenCV 。

如果不想使用 OpenCV ，你可编辑 `<sdk>/cmake/Option.cmake` 里的 `WITH_API` 选项，设为 `OFF` 就能关闭 `api` 层代码编译：

```cmake
option(WITH_API "Build with API layer, need OpenCV" ON)
```
