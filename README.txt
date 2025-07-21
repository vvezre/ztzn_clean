Git 全局设置:

git config --global user.name "陈大少"
git config --global user.email "1459422492@qq.com"
创建 git 仓库:

mkdir cleaner
cd cleaner
git init
touch README.md
git add README.md
git commit -m "first commit"
git remote add origin https://gitee.com/persionalPage/cleaner.git
git push -u origin "master"
已有仓库?

cd existing_git_repo
git remote add origin https://gitee.com/persionalPage/cleaner.git
git push -u origin "master"