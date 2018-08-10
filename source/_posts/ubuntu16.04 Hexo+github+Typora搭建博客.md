---
title: ubuntu16.04 Hexo+github+Typora搭建博客
date: 2018-03-21 22:22:50
tags: 
  - ubuntu16.04
  - Hexo 
  - Github
  - Typora
categories: Hexo
---

------

# **预备知识**

Hexo是一个基于Node.js的静态博客程序，可以方便的生成静态网页托管在github、gitcafe和Heroku上。博客存放的不只是文章内容，还有文章列表、分类、标签、翻页等动态内容，hexo所做的就是将这些md文件都放在本地，更新博文目录和相关链接信息，每次写完文章后调用写好的命令来批量完成相关页面的生成，然后再将有改动的页面提交到github。

Hexo依赖Node.js和Git。nvm（node version manager）是nodejs版本管理工具，管理nodejs和npm的版本；npm是随同nodeJs一起安装的包管理工具，npm管理对应nodeJs的第三方插件；nvm管理构建nodejs和对应的npm，npm管理对应的nodejs的第三方插件。

<!--more-->

# **本地搭建**

## 安装Git

~~~shell
sudo apt-get install git-core
~~~

## 安装Node.js

最好的方式是使用NVM（Node Version Manager）安装，在终端安装nvm执行命令：

cURL：

~~~sh
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.2/install.sh | bash
~~~

Wget：

~~~shelll
wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.33.2/install.sh | bash
~~~

重启终端安装Node.js：

~~~shell
nvm install stable
~~~

## 安装Hexo

~~~shell
npm install -g hexo-cli
~~~

## 初始化Hexo

~~~shell
mkdir git
cd git
hexo init hexo   #自定义的文件夹
cd hexo
npm install 
~~~

## 设置Hexo

执行命令：

~~~shell
hexo g/generate  #生成静态网页
hexo s/server    #运行本地服务器
~~~

如果出现提示：

~~~shell
INFO  Start processing
INFO  Hexo is running at http://localhost:4000/. Press Ctrl+C to stop.
~~~

说明安装成功，在浏览器地址栏输入http://localhost:4000 就可以看到默认主题的博客界面了。

## 博客主题更改

安装主题next或yillia，在～/git/hexo/目录下执行命令：

~~~shell
hexo clean
git clone https://github.com/iissnan/hexo-theme-next themes/next
~~~

或

~~~shell
git clone git@github.com:litten/hexo-theme-yilia.git themes/yilia
~~~

更新主题，修改hexo目录下的`_config.yml`，将`theme`属性设置为`next`或`yilia`，默认是landscape。

执行命令查看本地效果：

~~~shell
hexo g
hexo s
~~~

到此为止已经完成了Hexo博客的本地安装和查看，下一步是将博客部署到github上面，这样就可以通过网络远程访问自己的博客了。

目前使用的是next主题，完整的安装配置过程可以参考[这里](http://theme-next.iissnan.com/getting-started.html)。

# **部署到github**

- 首先到github上面注册自己的账号。
- 配置github

命令行输入命令：

~~~shell
git config --global user.name "username" #ruoxiangli
git config --global user.eamil "email@example.com" #981968690@qq.com
~~~

其中`yourname `是输入你自己的用户名，`email@example.com`输入你自己的注册邮箱。

这里可以使用`git config --list`命令查看配置好的内容（保存在home/.gitconfig文件中），如果需要修改用户名或邮箱，执行如下命令（也可以直接修改文件）：

~~~shell
git config --global --replace-all user.name “username”
git config --global --replace-all user.email “email@example.com”
~~~

- 创建公钥

命令行输入命令：

~~~shell
ssh-keygen -C 'you email address@gmail.com' -t rsa
~~~

**说明：C必须大写，改为自己的注册邮箱，然后一直回车，直到出现`“The key’s randomart image is：”`的提示。**

之后用户目录` ~/.ssh/ `下建立了相应的密钥文件`id_rsa.pub `，打开该文件。

- 添加公钥

github首页右上角点击头像，选择`Settings`，再选择`New SSH KEY`，把上一步`id_rsa.pub`文件的秘钥复制进去生成公钥。

- 创建项目仓库

github首页点击右上角的`+`，选择`New repository`。在页面里输入`username.github.io`，必须这么写。填完后点击`Create repository`。

- 部署博客

修改hexo目录下的`_config.yml`文件：

最后面修改为：

~~~yaml
deploy:
  type: git
  repository: git@github.com:username/username.github.io.git
  branch: master
~~~

安装hexo的插件：

~~~shell
npm install hexo-deployer-git --save
~~~

然后：

~~~shell
hexo clean
hexo generate
hexo deploy
~~~

在浏览器输入`yourname.github.io`就可以访问的自己的博客啦。
# 绑定自己的域名
绑定域名分2种情况：带www和不带www的。

域名配置最常见有2种方式，CNAME和A记录，CNAME填写域名，A记录填写IP，由于不带www方式只能采用A记录，所以必须先ping一下`yourname.github.io`的IP，然后到域名DNS设置页，将A记录指向ping出来的IP，将CNAME指向`username.github.io`，这样可以保证无论是否添加www都可以访问，如下：

![img](http://image.liuxianan.com/201608/20160823_191336_238_8683.png)

然后到github项目根目录新建一个名为CNAME的文件（无后缀），里面填写自己的域名。在绑定了新域名之后，原来的`username.github.io`并没有失效，还是会自动跳转到新域名。

# 编辑.md文件工具推荐

笔者使用的是Typora，[官网](https://typora.io/)，网页最下面是下载入口，根据自己的系统选择，ubuntu的方式如下：

~~~shell
# optional, but recommended
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BA300B7755AFCFAE
# add Typora's repository
sudo add-apt-repository 'deb http://typora.io linux/'
sudo apt-get update
# install typora
sudo apt-get install typora
~~~

Typora的Markdown语法的学习可以参考[博客](http://blog.csdn.net/tzs_1041218129/article/details/54728799)。

# 多机更新

使用坚果云同步hexo文件夹文件。





# 遇到的错误解决方法

发现执行时nvm install stable时出现未找到‘nvm’命令的错误提示，解决方式，分别执行下面两行指令：

~~~shell
export NVM_DIR="$HOME/.nvm"
[ -s "NVM_DIR/nvm.sh" ] && \. "NVM_DIR/nvm.sh"  # This loads nvm
~~~

执行`hexo server`后访问`http://localhost:4000`，出现`Cannot Get /`提示，打不开网页，可能是由于端口号4000被占用，可以使用其他端口号打开。解决方式：`hexo server -p 5000`
