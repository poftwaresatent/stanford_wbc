#include "dtor_check.hpp"
#include <execinfo.h>
#include <sstream>


namespace wbc {
  
  
  dtor_check::
  dtor_check()
    : previous_that(0)
  {
  }


  void dtor_check::
  check(void * that)
  {
    static int const maxsize(20);
    void *array[maxsize];
  
    if (previous_that) {
      int size = backtrace(array, maxsize);
      char ** strings = backtrace_symbols(array, size);
      fprintf(stderr,
	      "dtor_check::check(): GOTCHA BABY!\n"
	      "  current instance %08p\n"
	      "  current backtrace (%d levels):\n",
	      that, size);
      for (int ii(0); ii < size; ++ii)
	fprintf(stderr, "    %s\n", strings[ii]);
      free (strings);
      fprintf(stderr,
	      "  previous instance %08p\n"
	      "  previous backtrace:\n"
	      "%s"
	      "...going bonkers in a second...\n",
	      previous_that, previous_bt.c_str());
      usleep(1000000);
      ((void (*)()) 0)();
    }
    else {
      int size = backtrace(array, maxsize);
      char ** strings = backtrace_symbols(array, size);
      fprintf(stderr,
	      "dtor_check::check():  SAIVector %08p with data %08p is OK\n"
	      "  backtrace (%d levels):\n",
	      that, size);
      for (int ii(0); ii < size; ++ii)
	fprintf(stderr, "    %s\n", strings[ii]);
      fprintf(stderr, "  ...trying to save backtrace for future reference...\n");
      std::ostringstream os;
      for (int ii(0); ii < size; ++ii)
	os << "    " << strings[ii] << "\n";
      free (strings);
      previous_that = that;
      previous_bt = os.str();
      fprintf(stderr, "  saved backtrace:\n%s", previous_bt.c_str());
    }
  }
  
}
