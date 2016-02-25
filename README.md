# lirc_sunxi_rewrite
this driver is a backport of ir-driver present in the mainline branch in kernel 3.4.  for SoC for Allwinner A20.

It allows using the RC-CORE framework.

for now it is tested on a cubietruck.

this test is realised with yamaha RXV365 remote and samsung remote.

ir-keytable is able to report scancode.
