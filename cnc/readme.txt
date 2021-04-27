All FreeCAD files are created with version 0.18
All milling was done with 2mm diameter.

AudiomuxVer9-0 -> Backside milling for the connectors
exported nc file: backVer9-0.nc
exported frame: export-v9.pdf -> use to place the case and axis x:y zero at the right position. The center of the lower jack cutout is position 0:0.

AudiomuxFrontVer2-0 -> Front milling while using each LED.
Dont use. 1. The upper row of LEDs is 1.6mm too low (forgot the thickness of the PCB).
2. Starting milling in the edge of the holes ends in non round holes - it looks bad.
But this file is the only one which does the IR receiver cutout (at the righ position).

AudiomuxFrontVer5-0 -> Front milling of two windows for the two LED rows.
The larger height of the upper cutout is the result of fixing a previous made error. Reduce the height if you want. The center of origin is one of the LEDs.
exported nc file: ledFrontVer5-0.nc
export-FrontVer4-0.pdf is the outline on the case in the end (with the round corner from AudiomuxFrontVer2).
export-frontVer2-0.pdf can be used to place the milling at the right X-Y 0:0 position.

AudiomuxTransparent2Ver2-0 -> transparent window for the LED cutouts on the front (1x small, 1x large). You should use transparent plastic with 3.5 - 4mm thickness.
exported nc file: AudiomuxTransparent2Ver2-0.nc

AudiomuxTransparent3Ver2-0 -> transparent window for the IR receiver cutouts on the front
exported nc file: AudiomuxTransparent3Ver2-0.nc
