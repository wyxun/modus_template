# A stopped release build without the shell can discard all user start
# commands at link time. Keep just the real Arm entry for the dual-board
# restart replay; template.elf remains the shipping size/count reference.
$(BUILD_DIR)/bus-arm.elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(subst template.map,bus-arm.map,$(LDFLAGS)) -Wl,-u,mfoc_board_bus_Arm -o $@
