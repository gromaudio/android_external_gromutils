/*
 * Copyright (C) 2011 Davidlohr Bueso <dave@gnu.org>
 *
 * procutils.c: General purpose procfs parsing utilities
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Library Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library Public License for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <dirent.h>
#include <ctype.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/limits.h>

#include "procutils.h"
#include "all-io.h"

/*
 * @pid: process ID for which we want to obtain the threads group
 *
 * Returns: newly allocated tasks structure
 */
struct proc_tasks *proc_open_tasks(pid_t pid)
{
	struct proc_tasks *tasks;
	char path[PATH_MAX];

	sprintf(path, "/proc/%d/task/", pid);

	tasks = malloc(sizeof(struct proc_tasks));
	if (tasks) {
		tasks->dir = opendir(path);
		if (tasks->dir)
			return tasks;
	}

	free(tasks);
	return NULL;
}

/*
 * @tasks: allocated tasks structure
 *
 * Returns: nothing
 */
void proc_close_tasks(struct proc_tasks *tasks)
{
	if (tasks && tasks->dir)
		closedir(tasks->dir);
	free(tasks);
}

/*
 * @tasks: allocated task structure
 * @tid: [output] one of the thread IDs belonging to the thread group
 *        If when an error occurs, it is set to 0.
 *
 * Returns: 0 on success, 1 on end, -1 on failure or no more threads
 */
int proc_next_tid(struct proc_tasks *tasks, pid_t *tid)
{
	struct dirent *d;
	char *end;

	if (!tasks || !tid)
		return -EINVAL;

	*tid = 0;
	errno = 0;

	do {
		d = readdir(tasks->dir);
		if (!d)
			return errno ? -1 : 1;		/* error or end-of-dir */

		if (!isdigit((unsigned char) *d->d_name))
			continue;
		errno = 0;
		*tid = (pid_t) strtol(d->d_name, &end, 10);
		if (errno || d->d_name == end || (end && *end))
			return -1;

	} while (!*tid);

	return 0;
}

/* returns process command name, use free() for result */
char *proc_get_command(pid_t pid)
{
	char buf[BUFSIZ], *res = NULL;
	ssize_t sz = 0;
	size_t i;
	int fd = -1;

	snprintf(buf, sizeof(buf), "/proc/%d/cmdline", (int) pid);
	fd = open(buf, O_RDONLY);
	if (fd < 0)
		goto done;

	sz = read_all(fd, buf, sizeof(buf));
	if (sz <= 0)
		goto done;

	for (i = 0; i < (size_t) sz; i++) {

		if (buf[i] == '\0')
			buf[i] = ' ';
	}
	buf[sz - 1] = '\0';
	res = strdup(buf);
done:
	if (fd >= 0)
		close(fd);
	return res;
}

struct proc_processes *proc_open_processes(void)
{
	struct proc_processes *ps;

	ps = calloc(1, sizeof(struct proc_processes));
	if (ps) {
		ps->dir = opendir("/proc");
		if (ps->dir)
			return ps;
	}

	free(ps);
	return NULL;
}

void proc_close_processes(struct proc_processes *ps)
{
	if (ps && ps->dir)
		closedir(ps->dir);
	free(ps);
}

void proc_processes_filter_by_name(struct proc_processes *ps, const char *name)
{
	ps->fltr_name = name;
	ps->has_fltr_name = name ? 1 : 0;
}

void proc_processes_filter_by_uid(struct proc_processes *ps, uid_t uid)
{
	ps->fltr_uid = uid;
	ps->has_fltr_uid = 1;
}

int proc_next_pid(struct proc_processes *ps, pid_t *pid)
{
	struct dirent *d;

	if (!ps || !pid)
		return -EINVAL;

	*pid = 0;
	errno = 0;

	do {
		char buf[BUFSIZ], *p;

		errno = 0;
		d = readdir(ps->dir);
		if (!d)
			return errno ? -1 : 1;		/* error or end-of-dir */


		if (!isdigit((unsigned char) *d->d_name))
			continue;

		/* filter out by UID */
		if (ps->has_fltr_uid) {
			struct stat st;

			if (fstat_at(dirfd(ps->dir), "/proc", d->d_name, &st, 0))
				continue;
			if (ps->fltr_uid != st.st_uid)
				continue;
		}

		/* filter out by NAME */
		if (ps->has_fltr_name) {
			char procname[256];
			FILE *f;

			snprintf(buf, sizeof(buf), "%s/stat", d->d_name);
			f = fopen_at(dirfd(ps->dir), "/proc", buf,
						O_CLOEXEC|O_RDONLY, "r");
			if (!f)
				continue;

			p = fgets(buf, sizeof(buf), f);
			fclose(f);
			if (!p)
				continue;

			if (sscanf(buf, "%*d (%255[^)])", procname) != 1)
				continue;

			/* ok, we got the process name. */
			if (strcmp(procname, ps->fltr_name) != 0)
				continue;
		}

		p = NULL;
		errno = 0;
		*pid = (pid_t) strtol(d->d_name, &p, 10);
		if (errno || d->d_name == p || (p && *p))
			return errno ? -errno : -1;

		return 0;
	} while (1);

	return 0;
}

