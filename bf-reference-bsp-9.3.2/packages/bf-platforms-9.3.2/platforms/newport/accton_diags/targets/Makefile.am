ACLOCAL_AMFLAGS = $(ACLOCAL_FLAGS) -I m4

p4_targetdir = ${DESTDIR}/$(datadir)/p4/targets
target = "accton_newport_diag"
p4_target_DATA =
install-data-hook:
	mkdir -p $(p4_targetdir)/tofino2; \
	target_conf_filename=$(p4_targetdir)/tofino2/${target}.conf; \
	if [[ ! -e @abs_builddir@/tofino2/$${target}.conf.in ]]; then \
		cp @srcdir@/tofino2/${target}.conf.in $$target_conf_filename; \
	else \
		cp @abs_builddir@/tofino2/${target}.conf.in $$target_conf_filename; \
	fi
if IS_THRIFT_ENABLED
	:
else
	sed --in-place 's/^.*libpdthrift.so\", $$//' $(p4_targetdir)/tofino2/${target}.conf
endif
uninstall-hook:
	for target in $(P4PROGRAMS); do \
	  rm -f $(p4_targetdir)/tofino2/$${target}.conf; \
	done
