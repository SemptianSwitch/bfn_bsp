CFLAGS = -g 
CPPFLAGS = -I./install/include
LIBS = -lusb-1.0 -lcurl
LDFLAGS = -L./install/lib
cpmake: 
	gcc -o cp2112_util cp2112_util.c -lpltfm_driver -ldriver -ldru_sim -lbf_switchd_lib -lbfsys -lavago $(CFLAGS) $(CPPFLAGS) $(LIBS) $(LDFLAGS) 
