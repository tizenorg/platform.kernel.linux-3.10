Name: linux-kernel
Summary: The Linux Kernel
Version: 3.10.0
Release: 1
License: GPL
Group: System Environment/Kernel
Vendor: The Linux Community
URL: http://www.kernel.org
Source0:   %{name}-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{PACKAGE_VERSION}-root
BuildRequires: linux-glibc-devel
BuildRequires: git

%define abiver 1

%description
The Linux Kernel, the operating system core itself

%package headers
Summary: Header files for the Linux kernel for use by glibc
Group: Development/System
Obsoletes: kernel-headers
Provides: kernel-headers = %{version}

%description headers
Kernel-headers includes the C header files that specify the interface
between the Linux kernel and userspace libraries and programs.  The
header files define structures and constants that are needed for
building most standard programs and are also needed for rebuilding the
glibc package.

%package sources
Summary: Full linux kernel sources for out-of-tree modules
Group: Development/System
Provides: kernel-sources = %{version}-%{abiver}

%description sources
Full linux kernel sources for out-of-tree modules.

%package build
Summary: Prebuilt linux kernel for out-of-tree modules
Group: Development/System
Requires: kernel-sources = %{version}-%{abiver}

%description build
Prebuilt linux kernel for out-of-tree modules.


%prep
%setup -q

%build
mkdir -p %{buildroot}/usr/src/linux-kernel-sources-%{version}-%{abiver}
mkdir -p %{buildroot}/usr/src/linux-kernel-build-%{version}-%{abiver}

tar cpsf - . --one-file-system | tar xvfC - %{buildroot}/usr/src/linux-kernel-sources-%{version}-%{abiver}

make ARCH=arm EXTRAVERSION="-%{abiver}" O=%{buildroot}/usr/src/linux-kernel-build-%{version}-%{abiver} tizen_defconfig modules

%install
QA_SKIP_BUILD_ROOT="DO_NOT_WANT"; export QA_SKIP_BUILD_ROOT
mkdir $RPM_BUILD_ROOT/usr
make ARCH=arm INSTALL_HDR_PATH=$RPM_BUILD_ROOT/usr headers_install

ln -sf ../linux-kernel-source-%{version} %{buildroot}/usr/src/linux-kernel-build-%{version}-%{abiver}/source

%clean
rm -rf $RPM_BUILD_ROOT

%files headers
%defattr (-, root, root)
/usr/include

%files sources
%defattr (-, root, root)
/usr/src/linux-kernel-sources-*

%files build
%defattr (-, root, root)
/usr/src/linux-kernel-build-*
