CFLAGS = -g 
CPPFLAGS = -I./install/include
LDFLAGS = -L./install/lib
cpmake: 
	gcc -o fpga_util fpga_util.c -lpltfm_driver -ldriver -ldru_sim -lbf_switchd_lib -lbfsys -lavago $(CFLAGS) $(CPPFLAGS) $(LIBS) $(LDFLAGS) 
