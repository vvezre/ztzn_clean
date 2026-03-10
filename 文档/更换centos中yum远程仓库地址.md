### 更换centos中yum远程仓库地址

1. 修改`/etc/yum.repos.d/CentOS-Base.repo`，将所有`mirrorlist=`开头的行注释，并取消`baseurl=`的注释

    #mirrorlist=http://mirrorlist.centos.org/release=$releasever&ars
    baseurl=http://vault.centos.org/7.2.1511/os/$basearch/

2. 清理缓存并测试

    yum clean all
    yum makecache

3. 查看centos版本

    cat /etc/centos-release

