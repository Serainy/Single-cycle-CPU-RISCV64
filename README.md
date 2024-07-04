# Arch-2024-Spring
single-cycle-CPU-RISCV64
> 2024 Spring Computer Organization and Architecture(H) Course Code Repository.

Arch-2024-Sping  
│── build：仿真测试时才会生成的目录  
│── difftest：仿真测试框架  
│── ready-to-run：仿真测试文件目录（包括汇编文件和二进制文件等）  
│── verilate：verilator部分仿真文件目录  
│── vsrc：需要写的CPU代码所在目录  
│　　├── include：头文件目录  
│　　├── src：你将在这个目录下完成CPU的代码编写  
│　　　　　└── core.sv：CPU核主体代码  
│　　├── util：访存接口相关目录  
│　　└── SimTop.sv  
│── Makefile：仿真测试的命令汇总  
└── README.md: 此文件  

生成波形图：
make test-lab5 VOPT="--dump-wave"

Build:
  Linux 环境（推荐Ubuntu 20.04或更高版本）
  Verilator
  GTKWave
  Vivado 2018.3（其它版本可能会有不兼容）


1 Linux 环境安装
有三种方案：

Windows+WSL
Linux 虚拟机（VMware）
Linux 原生系统

如果你没安装过后两种，那么推荐第一种方案，以下给出安装步骤：

在 Windows 系统上打开 Microsoft Store
搜索 Ubuntu 20.04 并安装
安装 Visual Studio Code（推荐的代码编辑器）
配置 apt 源（修改 /etc/apt/sources.list ，推荐中科大源）
安装一些基本的软件包sudo apt install git perl python3 make autoconf g++ flex bison ccache



2 仿真器 Verilator
我们将使用 verilator 进行电路仿真。需要 verilator >= 4.200 版本。Verilator 没有 Windows 版本的，需要在 Linux 环境中运行。
无论你使用什么版本的 Linux，我们都推荐从源码进行编译。

cd ~ #如果使用 WSL，这个目录在 C 盘的某个位置
git clone git://github.com/verilator/verilator #卡了多试几次，或者换成https://gitlab.com/fudan-systa/verilator
cd verilator
git checkout v4.210
autoconf #如果显示 not found，那么sudo apt install autoconf后再执行 autoconf
./configure
make -j12  #如果这一步出现问题，改成make -j4或者直接make
sudo apt remove verilator #卸载已有的版本
sudo make uninstall
sudo make install
verilator --version  #查看verilator的版本


如果报某个头文件 not found，请参考 https://verilator.org/guide/latest/install.html#git-quick-install

3 波形图软件 GTKWave
我们将使用 GTKWave 软件查看仿真波形图。
如果你使用 Linux 虚拟机或 Linux 原生系统，只需：

sudo apt install gtkwave
运行 GTKwave： gtkwave trace.fst


如果你使用的是 Windows + WSL，则需要安装 GTKwave 的 Windows 版本，点击此处下载压缩文件，使用时，双击 fst 波形图文件，设置 gtkwave/bin/gtkwave.exe 为默认打开程序即可。
