#!/bin/bash
# BFN Platform debian packaging script with multi profile support

function print_help() {
    echo "USAGE: $(basename ""$0"") [OPTIONS]"
    echo "Options:"
    echo "  -p SWITCH PROFILE NAME"
    echo "    One option each for each profile to be packaged"
    echo "  --install-dir PATH"
    echo "    Path to folder with compiled profiles"
    echo "  --default-profile"
    echo "  Default Switch Profile (one of the profiles specified by -p option)"
    exit 0
}

opts=`getopt -o p:h --long default-profile:,install-dir: -- "$@"`

if [ $? != 0 ]; then
    exit 1
fi
eval set -- "$opts"

# Default Profile to use
DEFAULT_PROFILE=""
SCRIPT_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
CWD=$(pwd)

HELP=false
while true; do
    case "$1" in
        -p) PROFILES+=("$2"); shift 2;;
        -h) HELP=true; shift 1;;
        --default-profile) DEFAULT_PROFILE=$2; shift 2;;
        --install-dir) INSTALL_DIR="$(readlink -f "${2}")"; shift 2;;
        --) shift; break;;
    esac
done

if [ $HELP = true ]; then
    print_help
fi

if [ -v PROFILES ] && [ -z $DEFAULT_PROFILE ]; then
    echo "Must specify a default profile while packaging with -p option"
    print_help
fi

if [ -z ${SDE} ]; then
    path="${CWD}"
    while [ ${path} != / ]; do
        stat="$(find "${path}" -maxdepth 1 -mindepth 1 -iname "install" 2>/dev/null)"
        if [ "x${stat}" != "x" ] ; then
            SDE=${path}
            break
        fi

        stat="$(find "${path}" -maxdepth 1 -mindepth 1 -iname "install_*" 2>/dev/null)"
        if [ "x${stat}" != "x" ] ; then
            for item in ${stat} ; do
                profile="$(echo ${item} | awk -F 'install_' '{print $2}')"
                if [ ${PROFILES[*]} =~ ${profile} ] ; then
                    SDE=${path}
                    break
                fi
            done
        fi
        path="$(readlink -f "${path}"/..)"
    done
    if [ -z ${SDE} ]; then
        echo "SDE root directory not found."
        exit 1
    fi
fi
case ${CWD}/ in
    ${SDE}/install/*)
        echo "Can't create package at the current location"
        exit 1
    ;;
esac

echo "Found SDE root at ${SDE}"

if [ -z ${INSTALL_DIR} ]; then
    INSTALL_DIR=$SDE
    echo "Use SDE root as install dir"
else
    echo "Use ${INSTALL_DIR} as install dir"
fi

TMPDIR="$(mktemp -d -p /tmp)"

echo "Building package in ${TMPDIR}"

SDE_PKGNAME="bfnplatform"
# sudo apt-get install -y dh-make fakeroot
pushd ${TMPDIR}

make_links() {
    ln -s libpltfm_mgr.so.0.0.0 libpltfm_mgr.so.0
    ln -s libpltfm_mgr.so.0.0.0 libpltfm_mgr.so
    ln -s libtcl_server.so.0.0.0 libtcl_server.so.0
    ln -s libtcl_server.so.0.0.0 libtcl_server.so
    ln -s libpltfm_driver.so.0.0.0 libpltfm_driver.so.0
    ln -s libpltfm_driver.so.0.0.0 libpltfm_driver.so
    ln -s libbfsys.so.0.0.0 libbfsys.so.0
    ln -s libbfsys.so.0.0.0 libbfsys.so
	
	
	ln -s libtofbringup_driver.so.0.0.0 libtofbringup_driver.so.0
	ln -s libtofbringup_driver.so.0.0.0 libtofbringup_driver.so

    if [ -f libacctonbf_driver.so.0.0.0 ]; then
        ln -s libacctonbf_driver.so.0.0.0 libacctonbf_driver.so.0
        ln -s libacctonbf_driver.so.0.0.0 libacctonbf_driver.so
    fi

    if [ -f libnewport_driver.so.0.0.0 ]; then
        ln -s libnewport_driver.so.0.0.0 libnewport_driver.so.0
        ln -s libnewport_driver.so.0.0.0 libnewport_driver.so
    fi
}

BF_PLATFORMS=$SDE/submodules/bf-platforms
if [ ! -d $BF_PLATFORMS ]; then
    BF_PLATFORMS=$SDE/pkgsrc/bf-platforms
fi
LIB_DIR="-l"
package_profile() {
    local instdir
    local distdir
    if [ "$1" = "dummy" ]; then
        instdir="install"
        distdir="${INSTALL_DIR}/install"
        LIB_DIR+=":\$(shell pwd)/files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0"
    else
        instdir="install_$1"
        distdir="${INSTALL_DIR}/install_$1"
        LIB_DIR+=":\$(shell pwd)/files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0/"
    fi

    mkdir -p files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
	
	mkdir -p files/opt/bfn/${instdir}/share/platforms/tofino-bringup
	

    cp ${distdir}/lib/libpltfm_mgr.so.0.0.0 files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
    cp ${distdir}/lib/libpltfm_driver.so.0.0.0 files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
    cp ${distdir}/lib/libbfsys.so.0.0.0 files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0

    if [ -f ${distdir}/lib/libacctonbf_driver.so.0.0.0 ]; then
        cp ${distdir}/lib/libacctonbf_driver.so.0.0.0 files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
    fi

    if [ -f ${distdir}/lib/libnewport_driver.so.0.0.0 ]; then
        cp ${distdir}/lib/libnewport_driver.so.0.0.0 files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
    fi

    #local libthrift=`ldd ${distdir}/lib/libplatform_thrift.so.0.0.0|grep libthrift|cut -d" " -f1|cut -f2`
    if [ -n $libthrift ]; then
        cp /usr/local/lib/${libthrift} files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
    fi
	
	cp ${distdir}/share/platforms/tofino-bringup/board_lane_map.json files/opt/bfn/${instdir}/share/platforms/tofino-bringup/
    cp ${distdir}/lib/libtofbringup_driver.so.0.0.0 files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0

    cd files/opt/bfn/${instdir}/lib/platform/x86_64-semptian_ps7350_32x-r0
    make_links
    cd -
    cd files/opt/bfn/${instdir}/lib/platform
    cd -
}

if [ ! -v PROFILES ];
then
    echo "Packaging Default Profile"
    package_profile "dummy"
else
    echo "Packaging multiple profiles"
    for profile in "${PROFILES[@]}"; do
        echo "Packaging profile '${profile}'"
        package_profile "${profile}"
    done
fi

echo "Creating debian package"
dh_make --native --single --packagename ${SDE_PKGNAME}-semptian_9.3.2 --email semptianrd1@semptian.com -c apache -y
export TAR_OPTIONS==--warning=no-file-changed
export DEB_DH_SHLIBDEPS_ARGS_ALL=--dpkg-shlibdeps-params=--ignore-missing-info
echo "override_dh_shlibdeps:" >> debian/rules
echo "override_dh_usrlocal:" >> debian/rules
echo "	dh_shlibdeps $LIB_DIR --dpkg-shlibdeps-params=--ignore-missing-info" >> debian/rules
echo "  files/opt/* opt" > debian/install
echo "Creating debian package"
export DEB_DH_SHLIBDEPS_ARGS_ALL=--dpkg-shlibdeps-params=--ignore-missing-info
echo "override_dh_shlibdeps:" >> debian/rules
echo "	dh_shlibdeps $LIB_DIR -Xtcl -Xtk -- --ignore-missing-info" >> debian/rules
dpkg-buildpackage -uc -us

popd

if [ "x${KEEPTMP}" != "xyes" ]; then
    rm -rf ${TMPDIR}
fi

mv /tmp/${SDE_PKGNAME}*deb ${CWD}/