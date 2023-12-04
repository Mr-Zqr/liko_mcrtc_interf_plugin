![IMG_0422](https://github.com/Mr-Zqr/liko_mcrtc_interf_plugin/assets/62141613/f479d5d4-598f-4bc5-b54b-9a975db1e4c4)


mc_rtc new plugin template
==

This project is a template for a new plugin wihtin [mc_rtc]

It comes with:
- a CMake project that can build a plugin for [mc_rtc], the project can be put within [mc_rtc] source-tree for easier updates
- clang-format files
- automated GitHub Actions builds on three major platforms

Quick start
--

1. Renaming the controller from `LikoMcrtcInterfPlugin` to `MyPlugin`. In a shell (Git Bash on Windows, replace sed with gsed on macOS):

```bash
sed -i -e's/LikoMcrtcInterfPlugin/MyPlugin/g' `find . -type f`
git mv src/LikoMcrtcInterfPlugin.cpp src/MyPlugin.cpp
git mv src/LikoMcrtcInterfPlugin.h src/MyPlugin.h
git mv etc/LikoMcrtcInterfPlugin.in.yaml etc/MyPlugin.in.yaml
```

2. You can customize the project name in vcpkg.json as well, note that this must follow [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md)

3. Build and install the project

4. Run using your [mc_rtc] interface of choice, add `MyPlugin` to the `Plugins` configuration entry or enable the autoload option

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
