/*
 * chrt.c - manipulate a task's real-time attributes
 *
 * 27-Apr-2002: initial version - Robert Love <rml@tech9.net>
 * 04-May-2011: make it thread-aware - Davidlohr Bueso <dave@gnu.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Copyright (C) 2004 Robert Love
 */

#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "procutils.h"

#define UTIL_LINUX_VERSION  "1\n"
#define USAGE_SEPARATOR     "\n"
#define _(x)                x


static inline void errmsg(char doexit, int excode, char adderr, const char *fmt, ...)
{
  fprintf(stderr, "chrt: ");
  if (fmt != NULL) {
    va_list argp;
    va_start(argp, fmt);
    vfprintf(stderr, fmt, argp);
    va_end(argp);
    if (adderr)
      fprintf(stderr, ": ");
  }
  if (adderr)
    fprintf(stderr, "%m");
  fprintf(stderr, "\n");
  if (doexit)
    exit(excode);
}
#define err(E, FMT...)  errmsg(1, E, 1, FMT)
#define warnx(FMT...)   errmsg(0, 0, 0, FMT)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

static void show_usage(int rc)
{
  FILE *out = rc == EXIT_SUCCESS ? stdout : stderr;

  fputs(_("Show or change the real-time scheduling attributes of a process.\n"), out);
  fputs(USAGE_SEPARATOR, out);
  fputs(_("Set policy:\n"
  " chrt [options] <priority> <command> [<arg>...]\n"
  " chrt [options] -p <priority> <pid>\n"), out);
  fputs(USAGE_SEPARATOR, out);
  fputs(_("Get policy:\n"
  " chrt [options] -p <pid>\n"), out);

  fputs(USAGE_SEPARATOR, out);
  fputs(_("Policy options:\n"), out);
  fputs(_(" -f, --fifo           set policy to SCHED_FIFO\n"), out);
  fputs(_(" -o, --other          set policy to SCHED_OTHER\n"), out);
  fputs(_(" -r, --rr             set policy to SCHED_RR (default)\n"), out);

  fputs(USAGE_SEPARATOR, out);
  fputs(_("Other options:\n"), out);
  fputs(_(" -a, --all-tasks      operate on all the tasks (threads) for a given pid\n"), out);
  fputs(_(" -m, --max            show min and max valid priorities\n"), out);
  fputs(_(" -p, --pid            operate on existing given pid\n"), out);
  fputs(_(" -v, --verbose        display status information\n"), out);

  fputs(USAGE_SEPARATOR, out);

  exit(rc);
}

static void show_rt_info(pid_t pid, int isnew)
{
  struct sched_param sp;
  int policy;

  /* don't display "pid 0" as that is confusing */
  if (!pid)
    pid = getpid();

  policy = sched_getscheduler(pid);
  if (policy == -1)
    err(EXIT_FAILURE, _("failed to get pid %d's policy"), pid);

  if (isnew)
    printf(_("pid %d's new scheduling policy: "), pid);
  else
    printf(_("pid %d's current scheduling policy: "), pid);

  switch (policy) {
  case SCHED_OTHER:
    printf("SCHED_OTHER\n");
    break;
  case SCHED_FIFO:
    printf("SCHED_FIFO\n");
    break;
  case SCHED_RR:
    printf("SCHED_RR\n");
    break;
  default:
    warnx(_("unknown scheduling policy"));
  }

  if (sched_getparam(pid, &sp))
    err(EXIT_FAILURE, _("failed to get pid %d's attributes"), pid);

  if (isnew)
    printf(_("pid %d's new scheduling priority: %d\n"),
           pid, sp.sched_priority);
  else
    printf(_("pid %d's current scheduling priority: %d\n"),
           pid, sp.sched_priority);
}

static void show_min_max(void)
{
  unsigned long i;
  int policies[] = {
    SCHED_OTHER,
    SCHED_FIFO,
    SCHED_RR,
  };
  const char *names[] = {
    "OTHER",
    "FIFO",
    "RR",
  };

  for (i = 0; i < ARRAY_SIZE(policies); i++) {
    int max = sched_get_priority_max(policies[i]);
    int min = sched_get_priority_min(policies[i]);

    if (max >= 0 && min >= 0)
      printf(_("SCHED_%s min/max priority\t: %d/%d\n"),
          names[i], min, max);
    else
      printf(_("SCHED_%s not supported?\n"), names[i]);
  }
}

int main(int argc, char **argv)
{
  int i, policy = SCHED_RR, priority = 0, verbose = 0, policy_flag = 0,
      all_tasks = 0;
  struct sched_param sp;
  pid_t pid = -1;

  static const struct option longopts[] = {
    { "all-tasks",  0, NULL, 'a' },
    { "fifo", 0, NULL, 'f' },
    { "pid",  0, NULL, 'p' },
    { "help", 0, NULL, 'h' },
    { "max",        0, NULL, 'm' },
    { "other",  0, NULL, 'o' },
    { "rr",   0, NULL, 'r' },
    { "verbose",  0, NULL, 'v' },
    { "version",  0, NULL, 'V' },
    { NULL,   0, NULL, 0 }
  };

  while((i = getopt_long(argc, argv, "+afphmorvV", longopts, NULL)) != -1)
  {
    int ret = EXIT_FAILURE;

    switch (i) {
    case 'a':
      all_tasks = 1;
      break;
    case 'f':
      policy = SCHED_FIFO;
      break;
    case 'm':
      show_min_max();
      return EXIT_SUCCESS;
    case 'o':
      policy = SCHED_OTHER;
      break;
    case 'p':
      errno = 0;
      pid = atoi(argv[argc - 1]);
      break;
    case 'r':
      policy = SCHED_RR;
      break;
    case 'v':
      verbose = 1;
      break;
    case 'V':
      printf(UTIL_LINUX_VERSION);
      return EXIT_SUCCESS;
    case 'h':
      ret = EXIT_SUCCESS;
      /* fallthrough */
    default:
      show_usage(ret);
    }
  }

  if (((pid > -1) && argc - optind < 1) ||
      ((pid == -1) && argc - optind < 2))
    show_usage(EXIT_FAILURE);

  if ((pid > -1) && (verbose || argc - optind == 1)) {
    if (all_tasks) {
      pid_t tid;
      struct proc_tasks *ts = proc_open_tasks(pid);

      if (!ts)
        err(EXIT_FAILURE, _("cannot obtain the list of tasks"));
      while (!proc_next_tid(ts, &tid))
        show_rt_info(tid, false);
      proc_close_tasks(ts);
    } else
      show_rt_info(pid, true);

    if (argc - optind == 1)
      return EXIT_SUCCESS;
  }

  errno = 0;
  priority = atoi(argv[optind]);

  policy |= policy_flag;

  if (pid == -1)
    pid = 0;
  sp.sched_priority = priority;

  if (all_tasks) {
    pid_t tid;
    struct proc_tasks *ts = proc_open_tasks(pid);

    if (!ts)
      err(EXIT_FAILURE, _("cannot obtain the list of tasks"));
    while (!proc_next_tid(ts, &tid))
      if (sched_setscheduler(tid, policy, &sp) == -1)
        err(EXIT_FAILURE, _("failed to set tid %d's policy"), tid);
    proc_close_tasks(ts);
  } else if (sched_setscheduler(pid, policy, &sp) == -1)
    err(EXIT_FAILURE, _("failed to set pid %d's policy"), pid);

  if (verbose)
    show_rt_info(pid, true);

  if (!pid) {
    argv += optind + 1;
    execvp(argv[0], argv);
    err(EXIT_FAILURE, _("failed to execute %s"), argv[0]);
  }

  return EXIT_SUCCESS;
}
