/* main.cc */

#include "reader.h"
#include "timer.h"
#include "planner.h"
#include <string.h>

#define VERSION "1.1"
#define DEBUG

// functions
void print_usage(char*);

Reader reader;
Timer timer;

int main(int argc, char **argv)
{
  int i;
  bool b_plan = false;
  bool b_semantics = false;
  bool b_debug = false;

  Planner planner(&reader, &timer);

  planner.completedinfo = false;
  cout << "CPA version " << VERSION << endl;

  if (argc < 2)
    print_usage(argv[0]);

  i = 2;
  while (i < argc)
  {
    if (strcmp(argv[i], "-pc") == 0 || strcmp(argv[i], "-ph") == 0)
    {
      if (b_semantics)
        print_usage(argv[0]);

      if (strcmp(argv[i], "-ph") == 0)
        planner.m_semantics = PH;
      else
        planner.m_semantics = PC;

      // stop reading plan
      b_plan = false;

      // mark the option
      b_semantics = true;
    }
    else if (strcmp(argv[i], "-e") == 0)
    {
      if (b_debug)
        print_usage(argv[0]);

      planner.m_task = DOPLAN;

      // mark this option
      b_debug = true;

      // start reading plan
      b_plan = true;
    }
    else if (b_plan)
    {
      planner.m_plan.push_back(string(argv[i]));
    }
    else if (strcmp(argv[i], "-c") == 0)
    {
      planner.completedinfo = true;
    }
    else
      print_usage(argv[0]);
    ++i;
  }

  if (freopen(argv[1], "r", stdin) == NULL)
  {
    cerr << argv[0] << ": File " << argv[1] << " cannot be opened.\n";
    exit(1);
  }

  timer.start(READ_TIMER);
  reader.read();
  timer.end(READ_TIMER);

  planner.main();

  exit(0);
}

void print_usage(char* prog_name)
{
  cout << "USAGE:" << endl;
  cout << "  " << prog_name << " input_domain [options]" << endl << endl;
  cout << "OPTIONS:" << endl;
  cout << "  -[pc|ph]" << endl;
  cout << "     Select an approximation to be used: pc for" << endl;
  cout << "     the possible-change approximation; ph for " << endl;
  cout << "     the possible-hold approximation." << endl << endl;
  cout << "  -e action1 action2 action3 ..." << endl;
  cout << "     Perform a sequence of actions and print out" << endl;
  cout << "     results step by step. The planner does not" << endl;
  cout << "     search for a plan." << endl << endl;
  cout << "  -c " << endl;
  cout << "     Generate a completed initial state" << endl;
  cout << "EXAMPLES:" << endl;
  cout << "  " << prog_name << " blw.al -ph" << endl;
  cout << "     Search for a conformant plan using the ph-approximation" << endl;
  cout << "     The input domain is contained in the blw.al file." << endl << endl;
  cout << "  " << prog_name << " blw.al -pc -e \"move(2,table)\" \"move(1,2)\"" << endl;
  cout << "     Execute the plan [move(2,table);move(1,2)] using the" << endl;
  cout << "     pc-approximation and print out results step by step. " << endl << endl;
  exit(1);
}
