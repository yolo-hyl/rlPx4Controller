# 获取当前安装路径
pkg_root_path=$(pip list -v|grep rlPx4Controller|awk '{print $3}')

# 删除生成的 .pyi 文件
rm -f ${pkg_root_path}/rlPx4Controller/pyControl.pyi
rm -f ${pkg_root_path}/rlPx4Controller/pyParallelControl.pyi
rm -f ${pkg_root_path}/rlPx4Controller/traj_tools/polyTrajGen.pyi

# 或者直接删除整个包目录（更彻底）
rm -rf ${pkg_root_path}/rlPx4Controller