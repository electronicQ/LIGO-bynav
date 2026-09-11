#!/bin/bash
set -e

# ==========================================
# LIGO 项目依赖一键安装脚本 (联网版)
# 适用环境: Ubuntu 20.04 + CMake 3.16
# 用法: chmod +x install_ligo_deps.sh && ./install_ligo_deps.sh
# ==========================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ---------- 第一部分: apt 安装系统依赖 ----------
APT_PACKAGES=(
    build-essential
    cmake
    git
    libomp-dev        # OpenMP
    python3-dev       # PythonLibs
    libopencv-dev     # OpenCV
    libeigen3-dev     # Eigen3
    libpcl-dev        # PCL 1.8+
    libfmt-dev        # Sophus 隐式依赖
    libdw-dev         # GTSAM/Ceres Profiling 依赖 (libdw)
)

log_info "正在通过 apt 安装系统依赖..."
sudo apt-get update
sudo apt-get install -y "${APT_PACKAGES[@]}" || log_error "apt 安装失败，请检查网络连接或软件源"
log_info "apt 依赖安装完成 ✅"

# ---------- 第二部分: 源码编译指定版本依赖 ----------
BUILD_DIR="$HOME/ligo_deps_src"
INSTALL_PREFIX="/usr/local"
NPROC=$(nproc)

mkdir -p "$BUILD_DIR"

# --- 通用源码编译函数 ---
build_from_source() {
    local name=$1
    local repo_url=$2
    local tag=$3
    local cmake_opts=$4
    local build_subdir="build"
    local src_dir="$BUILD_DIR/$name"

    if [ -d "$src_dir/.git" ]; then
        log_warn "$name 源码已存在，跳过克隆，直接重新编译..."
    else
        log_info "正在克隆 $name ($tag)..."
        git clone --depth 1 --branch "$tag" "$repo_url" "$src_dir" \

            || log_error "克隆 $name 失败，请检查网络或仓库地址"
    fi

    log_info "正在编译安装 $name ($tag)..."
    rm -rf "$src_dir/$build_subdir"
    mkdir -p "$src_dir/$build_subdir"
    cd "$src_dir/$build_subdir"

    cmake .. -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" $cmake_opts \

        || log_error "$name cmake 配置失败"
    make -j"$NPROC" \

        || log_error "$name 编译失败"
    sudo make install \

        || log_error "$name 安装失败"

    log_info "$name ($tag) 安装完成 ✅"
}

# --- GTSAM 4.2 ---
build_from_source "gtsam" \
    "https://github.com/borglab/gtsam.git" \
    "4.2" \
    "-DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_ENABLE_PROFILING=OFF"

# --- Ceres Solver 2.1.0 ---
build_from_source "ceres-solver" \
    "https://github.com/ceres-solver/ceres-solver.git" \
    "2.1.0" \
    "-DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DUSE_CUDA=OFF"

# --- Sophus 1.22.10 ---
build_from_source "Sophus" \
    "https://github.com/strasdat/Sophus.git" \
    "1.22.10" \
    "-DBUILD_TESTS=OFF"

# ---------- 刷新动态链接库缓存 ----------
log_info "正在更新 ldconfig..."
sudo ldconfig

# ---------- 验证安装 ----------
log_info "========== 安装验证 =========="
echo "GTSAM:   $(pkg-config --modversion gtsam 2>/dev/null || echo '未找到 pkg-config')"
echo "Ceres:   $(pkg-config --modversion ceres 2>/dev/null || echo '未找到 pkg-config')"
echo "Eigen3:  $(dpkg -s libeigen3-dev 2>/dev/null | grep Version | awk '{print $2}')"
echo "fmt:     $(dpkg -s libfmt-dev 2>/dev/null | grep Version | awk '{print $2}')"
echo "PCL:     $(dpkg -s libpcl-dev 2>/dev/null | grep Version | awk '{print $2}')"
echo "OpenCV:  $(dpkg -s libopencv-dev 2>/dev/null | grep Version | awk '{print $2}')"
echo "libdw:   $(dpkg -s libdw-dev 2>/dev/null | grep Version | awk '{print $2}')"

log_info "=========================================="
log_info "🎉 所有依赖安装完成！现在可以编译 LIGO 项目:"
log_info "   cd ~/LIGO_ws && catkin_make -j1"
log_info "=========================================="
