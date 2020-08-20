#include <iostream>
#include <dirent.h>
#include <sys/types.h>

using namespace std;
void list_dir(const char *path) {
   struct dirent *entry;
   DIR *dir = opendir(path);
   
   if (dir == NULL) {
      return;
   }
   int count = 0;
   while ((entry = readdir(dir)) != NULL) {
   cout << entry->d_name << endl;
   count ++;
   }
   closedir(dir);
   std::cout << "Final count  " << count << std::endl;
}
int main() {
   list_dir("/home/nvidia/catkin_ws/src/robot_diagnostics/include");
}
