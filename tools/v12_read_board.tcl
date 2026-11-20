# Run from the evidence directory; read only, never halt/reset the target.
source [find interface/stlink.cfg]
source [find target/stm32h7x.cfg]
gdb port disabled
tcl port disabled
telnet port disabled
adapter speed 4000
reset_config none
stm32h7x.cpu0 configure -event examine-end {} -work-area-size 0
init
targets stm32h7x.cpu0
poll
set uid [read_memory 0x1ff1e800 32 3]
set id [lindex [read_memory 0x5c001000 32 1] 0]
if {$id != 0x10016483 || [lindex $uid 0] != 0x420030 ||
    [lindex $uid 1] != 0x31345106 || [lindex $uid 2] != 0x38373332} {error WRONG_BOARD}
if {[stm32h7x.cpu0 curstate] ne "running"} {error NOT_RUNNING}
echo "TARGET $id UID $uid"
if {[info exists v12_inspect]} {
    source $v12_inspect
} else {
    # Interpret old RAM only after matching the backup to the known old image.
    echo "OLD_TASK [read_memory 0x20002bd4 8 2] OLD_J3 [read_memory 0x20005bdc 8 2]"
    for {set i 0} {$i < 5} {incr i} {
        echo "MOTOR_$i [read_memory [expr {0x20005c70+60*$i}] 32 15]"
    }
    dump_image before_flash_1MiB.bin 0x08000000 0x100000
    echo BACKED_UP_RUNNING
}
shutdown
