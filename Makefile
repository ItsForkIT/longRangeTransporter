build:
	@echo "Compiling stream.c ..."
	gcc sender/stream.c -o dist/stream -Isender
	@echo "Compiling gw.c ..."
	gcc receiver/gw.c -o dist/gw -Ireceiver -lm -lpthread
	@echo "Compiling wsnd.c ..."
	gcc wsnd/wsnd.c -o dist/wsnd -Iwsnd -lm -lpthread
	@echo "Finished"