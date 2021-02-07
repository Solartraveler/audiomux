all:
	make -j -C src-device

clean:
	make -C src-device clean

package:
	${CURDIR}/src-host/makeDebianPackage.sh

test:
	${CURDIR}/src-host/audiomux-control.py --help
