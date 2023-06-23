#include "util.h"
#include "stdlib.h"
int data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

#define LOOP_MAX 1000

int main(int argc, char* argv[]) {
    int sum = 0;

    for (int i = 0; i < LOOP_MAX; i++) sum += i;

    if (sum == (LOOP_MAX * (LOOP_MAX - 1) / 2)) exit(0);
    else exit(sum);

}