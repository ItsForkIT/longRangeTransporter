build:
	@echo "Compiling stream.c ..."
	gcc sender/stream.c -o dist/stream -Isender
	gcc receiver/gw.c -o dist/gw -Ireceiver -lm -lpthread
	@echo "Finished"