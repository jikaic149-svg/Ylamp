# Run only from the gated current V1.2 five-axis build, after backup and host
# checks, with mechanical support in place. No mass erase or motor commands.
set reset_connect [expr {[info exists ::env(V12_CONNECT_RESET)] && $::env(V12_CONNECT_RESET) eq "1"}]
if {$reset_connect && (![info exists ::env(V12_BACKUP_DIR)] || ![info exists ::env(V12_EXPECT_OLD)])} {
    error RESET_CONNECT_REQUIRES_BACKUP_AND_EXPECTED_OLD_IMAGE
}
if {[info exists ::env(V12_BACKUP_DIR)]} {
    set backup_dir $::env(V12_BACKUP_DIR)
    if {![file isdirectory $backup_dir] ||
        [file exists [file join $backup_dir before_flash_1MiB.bin]]} {error BAD_OR_USED_BACKUP_DIRECTORY}
}
source [find interface/stlink.cfg]
source [find target/stm32h7x.cfg]
gdb port disabled
tcl port disabled
telnet port disabled
if {$reset_connect} {
    # Explicitly authorized interruption only; requires a connected NRST.
    adapter speed 1000
    reset_config srst_only srst_nogate connect_assert_srst
} else {
    adapter speed 4000
    reset_config none
}
# The installed non-HLA H7 config accesses DBGMCU through AP2, including
# under SRST. Keep its debug clocks/low-power/watchdog setup for reset
# connection; clearing that hook can prevent a sleeping core from halting.
if {$reset_connect} {
    if {[using_hla]} {error RESET_CONNECT_REQUIRES_AP2_CAPABLE_ADAPTER}
} else {
    stm32h7x.cpu0 configure -event examine-end {}
}
stm32h7x.cpu0 configure -work-area-backup 1
init
targets stm32h7x.cpu0
if {$reset_connect} {
    # Connect under SRST, then use the usual direct halt after the UID check.
    # A reset-halt vector catch cannot work when the probe's NRST is not wired.
    adapter deassert srst
    reset_config none
}
poll
set uid [read_memory 0x1ff1e800 32 3]
set id [lindex [read_memory 0x5c001000 32 1] 0]
if {$id != 0x10016483 || [lindex $uid 0] != 0x420030 ||
    [lindex $uid 1] != 0x31345106 || [lindex $uid 2] != 0x38373332} {error WRONG_BOARD}
echo "TARGET $id UID $uid"
set image ylamp_v12_five_axis.bin
set length [file size $image]
if {$length < 0x10000 || $length > 0x100000} {error WRONG_SIZE}
halt
if {[info exists backup_dir]} {
    dump_image [file join $backup_dir before_flash_1MiB.bin] 0x08000000 0x100000
    if {[file size [file join $backup_dir before_flash_1MiB.bin]] != 0x100000} {error INCOMPLETE_BACKUP}
    echo "V12_BACKUP $backup_dir"
}
if {[info exists ::env(V12_EXPECT_OLD)]} {
    # Abort before erasing if the real old application differs from its archive.
    verify_image $::env(V12_EXPECT_OLD) 0x08000000 bin
    echo V12_OLD_IMAGE_VERIFIED
}
flash probe 0
flash write_image erase $image 0x08000000 bin
verify_image $image 0x08000000 bin
dump_image readback_v12.bin 0x08000000 $length
echo "V12_VERIFIED_BYTES $length"
cortex_m reset_config sysresetreq
reset run
poll
echo "AFTER_FLASH [stm32h7x.cpu0 curstate]"
shutdown
