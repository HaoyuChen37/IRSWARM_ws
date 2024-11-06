# multi-light-detecting system
## Initialize raspberry system
#### Download vscode
```bash
sudo apt update
sudo apt install code
```
#### Download vim
```bash
sudo apt install vim
```
#### Download miniconda
##### 获取miniconda安装包
```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-py37_4.9.2-Linux-aarch64.sh
```
##### 安装
```bash
bash Miniconda3-py37_4.9.2-Linux-aarch64.sh
```
##### 添加环境变量

要在当前用户下使用conda，那么还必须把他的执行路径添加进环境变量。
```bash
vim ~/.bashrc
```
进入.bashrc文件，如果提示没有vim的话用nano ~/.bashrc也可以。

在文件的最后添加一行：
```bash
export PATH="/home/pi/miniconda3/bin:$PATH"
```
这里的pi是用户名字

##### 使用
```bash
conda list
```

#### Create miniconda vitual environment
```bash
conda create --name light python=3.9
```

#### Download packages
```bash
pip install opencv-python -i https://mirrors.aliyun.com/pypi/simple/
```
