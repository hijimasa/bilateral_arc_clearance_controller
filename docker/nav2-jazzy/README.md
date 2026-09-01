# ROS 2 Jazzy / Nav2 build verification

This container provides a clean build-and-test environment for the complete
package, including the `nav2_core::Controller` plugin. It follows the same
`ros:jazzy-ros-base` and `rosdep` dependency resolution used by the hosted CI.
It is intentionally separate from the ROS 2 Humble Gazebo evidence container,
which builds only the framework-independent core and filter node.

From the package root, run:

```bash
./docker/nav2-jazzy/verify.sh
```

The script builds `bac-nav2-jazzy-verification`, mounts the checkout read-only,
and performs a Release `colcon build`, all package tests, and an installed Nav2
plugin-description check in a temporary container workspace. No `build`,
`install`, or `log` directories are written into the checkout.

Set `BAC_NAV2_IMAGE` to use another local image tag:

```bash
BAC_NAV2_IMAGE=my-bac-jazzy ./docker/nav2-jazzy/verify.sh
```

## 日本語

このコンテナは、`nav2_core::Controller`プラグインを含むパッケージ全体を、
クリーンなROS 2 Jazzy環境でビルド・テストするためのものです。hosted CIと同じ
`ros:jazzy-ros-base`および`rosdep`による依存解決を使用します。ROS 2 Humbleの
Gazebo evidence環境はcoreとfilter nodeのみを対象とするため、用途を分けています。

パッケージルートで`./docker/nav2-jazzy/verify.sh`を実行すると、ソースをread-onlyで
mountし、Releaseビルド、全テスト、インストール済みplugin descriptionの存在確認を
一時コンテナ内で行います。checkoutに`build`、`install`、`log`は生成しません。
