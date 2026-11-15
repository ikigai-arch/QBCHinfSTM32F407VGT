# Harness uji PC firmware QBC v2. gcc (Linux/WSL/MSYS2/MinGW). Jalankan: make test
SRC = ..
FW  = $(SRC)/qbc_core.c $(SRC)/ctrl_mux.c $(SRC)/qbc_sim.c $(SRC)/pi_ctrl.c $(SRC)/controller_step.c
CFLAGS = -std=c99 -O2 -Wall -Wextra -Wshadow -Wdouble-promotion -Wfloat-conversion -I$(SRC)
VEC = "../../../../MATLAB File_Hinfinit/MIL/Sil tahap akhir"

all: hasil harness thv

hasil:
	mkdir hasil

harness: harness.c $(FW)
	$(CC) $(CFLAGS) -Wno-float-conversion -Wno-double-promotion harness.c $(FW) -lm -o $@

coeffs_double.h: $(SRC)/controller_coeffs.h
	sed -e 's/\([0-9]\)f\b/\1/g' -e 's/static const float/static const double/' -e 's/CONTROLLER_COEFFS_H/CONTROLLER_COEFFS_D_H/' $< > $@

thv: test_hinf_vectors.c coeffs_double.h $(SRC)/controller_step.c $(SRC)/ctrl_mux.c $(SRC)/pi_ctrl.c
	$(CC) -std=c99 -O2 -Wall -I$(SRC) -I. test_hinf_vectors.c $(SRC)/controller_step.c $(SRC)/ctrl_mux.c $(SRC)/pi_ctrl.c -lm -o $@

test: all
	./thv $(VEC)/e_test.csv $(VEC)/y_matlab_test.csv | tee hasil/hasil_hinf_vectors.txt
	./harness

# kompilasi firmware portabel dengan peringatan penuh (mendeteksi double tak sengaja di ISR)
warn:
	$(CC) $(CFLAGS) -fsyntax-only $(SRC)/qbc_core.c $(SRC)/ctrl_mux.c $(SRC)/qbc_sim.c

clean:
	rm -f harness thv coeffs_double.h
