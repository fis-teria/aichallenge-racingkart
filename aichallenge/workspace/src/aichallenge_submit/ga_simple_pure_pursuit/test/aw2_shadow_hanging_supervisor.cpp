#include <csignal>
#include <unistd.h>

int main() {
  for (;;) {
    pause();
  }
}
