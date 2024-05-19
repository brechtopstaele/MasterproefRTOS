#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <overwrite.h>
#include "cmsis_os.h"

// Function to overwrite
void funcToOverwrite(){
	char output[10] = "Hallo";
	printf("Output: %s", output);
}
