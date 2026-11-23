# Companion link for cold and warm phase replay of shell-free builds.
# Keep the Port entries from the same objects used by template.elf.
$(BUILD_DIR)/pwm-phase.elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(subst template.map,pwm-phase.map,$(LDFLAGS)) \
	  -Wl,-u,mfoc_port_pwm_start,-u,mfoc_port_pwm_stop \
	  -Wl,-u,mfoc_port_current,-u,mfoc_port_pwm -o $@
