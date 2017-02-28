build:
	@echo "Compiling stream.c ..."
	gcc sender/stream.c -o dist/stream -Isender
	@echo "Finished"