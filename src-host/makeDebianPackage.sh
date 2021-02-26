#!/bin/bash

set -e

BASE=`pwd`

NAME="audiomux-control"
BINARY="${BASE}/${NAME}"
PACKAGEDIR="${BASE}/packages"
CONTROLFILE="${PACKAGEDIR}/DEBIAN/control"
VERSION=`${BINARY}.py --version | head -n1 | cut -d " " -f2`

mkdir -p "${PACKAGEDIR}/usr/bin"
cp "${BINARY}.py" "${PACKAGEDIR}/usr/bin/${NAME}"
chmod a+x "${PACKAGEDIR}/usr/bin/${NAME}"
mkdir -p "${PACKAGEDIR}/etc/udev/rules.d"
cp "${BASE}/55-audiomux.rules" "${PACKAGEDIR}/etc/udev/rules.d/"
mkdir -p "${PACKAGEDIR}/usr/share/doc/${NAME}"
cp "${BASE}/LICENSE" "${PACKAGEDIR}/usr/share/doc/${NAME}/copyright"

rm -f ${CONTROLFILE}

BINARYBYTES=`du -s -k ${PACKAGEDIR} | cut -f1 `

#Its a python script, so architecture is not important
ARCHITECTURE=all

echo "Package: audiomux-control" > ${CONTROLFILE}
echo "Version: ${VERSION}" >> ${CONTROLFILE}
echo "Priority: optional" >> ${CONTROLFILE}
echo "Architecture: ${ARCHITECTURE}" >> ${CONTROLFILE}
echo "Essential: no" >> ${CONTROLFILE}
echo "Installed-size: ${BINARYBYTES}" >> ${CONTROLFILE}
echo "Maintainer: Malte Marwedel" >> ${CONTROLFILE}
echo "Homepage: https://github.com/Solartraveler/audiomux" >> ${CONTROLFILE}
echo "Description: Program for controlling USB audiomuxer." >> ${CONTROLFILE}
echo "Depends: python3, python3-usb" >> ${CONTROLFILE}
dpkg-deb --root-owner-group --build "${PACKAGEDIR}" "${BASE}/../${NAME}_${VERSION}_${ARCHITECTURE}.deb"
