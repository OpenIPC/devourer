/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * The diagnostic sink (mt7612u_set_log_sink).
 *
 * This library used to write every diagnostic straight to stderr with
 * devourer's line format baked in. That is right for the bring-up tool and
 * wrong for anything embedding the library: it bypasses the host's log level,
 * bypasses a redirected diagnostic stream, and on Android bypasses
 * __android_log_write, so the lines land nowhere a user can see them.
 *
 * What matters here is that installing a sink actually DIVERTS — a hook that
 * receives a copy while stderr keeps getting the original would look identical
 * in casual use and fix nothing. So each case checks the sink saw the line AND
 * that stderr did not.
 */
#include <stdio.h>
#include <string.h>

#include "internal.h"

static int fails;

static void check(int ok, const char *what)
{
	if (!ok) {
		printf("  FAIL %s\n", what);
		fails++;
	}
}

/* --- the sink under test ------------------------------------------------ */

static char last_line[512];
static char last_level;
static int  calls;
static void *last_user;

static void capture(void *user, char level, const char *line)
{
	last_user = user;
	last_level = level;
	calls++;
	snprintf(last_line, sizeof last_line, "%s", line);
}

/* Redirect stderr to a temp file so "did stderr get anything?" is answerable.
 * freopen, not dup2: this has to work the same on Windows, where the library is
 * now built too. */
static FILE *steal_stderr(const char *path)
{
	return freopen(path, "w+", stderr);
}

static long stderr_len(void)
{
	long n;

	fflush(stderr);
	n = ftell(stderr);
	return n < 0 ? 0 : n;
}

int main(void)
{
	const char *path = "log_sink_stderr.tmp";
	int marker = 4242;

	printf("mt7612u log sink:\n");

	if (!steal_stderr(path)) {
		printf("  FAIL cannot redirect stderr\n");
		return 1;
	}

	/* 1. Default: the built-in sink writes, and it carries devourer's prefix
	 *    and the level letter. Nothing about the standalone tool changes. */
	mt_diag('I', "hello %d", 7);
	{
		long n = stderr_len();
		char buf[512];

		check(n > 0, "default sink writes to stderr");
		rewind(stderr);
		buf[0] = '\0';
		if (fgets(buf, sizeof buf, stderr) == NULL)
			buf[0] = '\0';
		check(strstr(buf, "devourer [I] mt7612u: hello 7") != NULL,
		      "default sink keeps devourer's line format");
		check(strchr(buf, '\n') != NULL, "default sink terminates the line");
	}

	/* 2. With a sink installed, the line goes THERE and stderr stays silent. */
	if (!steal_stderr(path)) {
		printf("  FAIL cannot re-redirect stderr\n");
		return 1;
	}
	mt7612u_set_log_sink(capture, &marker);
	calls = 0;
	mt_diag('W', "diverted %s", "line");
	check(calls == 1, "sink received exactly one call");
	check(last_level == 'W', "sink received the level letter");
	check(last_user == &marker, "sink received its user pointer");
	check(strcmp(last_line, "diverted line") == 0,
	      "sink receives the bare message, with no devourer prefix");
	check(stderr_len() == 0, "stderr stays silent while a sink is installed");

	/* A prefix reaching the sink would double up once a host adds its own. */
	check(strstr(last_line, "devourer") == NULL, "no prefix leaks to the sink");
	check(strchr(last_line, '\n') == NULL, "no newline leaks to the sink");

	/* 3. Every level letter reaches the sink unchanged - a sink that only saw
	 *    errors would silently drop bring-up progress. */
	{
		const char *levels = "IWE";
		size_t i;

		for (i = 0; i < strlen(levels); i++) {
			calls = 0;
			mt_diag(levels[i], "lvl");
			check(calls == 1 && last_level == levels[i],
			      "each level letter reaches the sink");
		}
	}

	/* 4. NULL restores the built-in sink rather than silencing the library,
	 *    so a host that tears its logger down does not lose diagnostics. */
	if (!steal_stderr(path)) {
		printf("  FAIL cannot re-redirect stderr\n");
		return 1;
	}
	mt7612u_set_log_sink(NULL, NULL);
	calls = 0;
	mt_diag('E', "back to stderr");
	check(calls == 0, "the removed sink is not called");
	check(stderr_len() > 0, "NULL restores the built-in stderr sink");

	/* 5. A long line must be truncated, not overrun. The formatted message
	 *    buffer is 512, so ask for more than that and require a bounded,
	 *    NUL-terminated result. */
	mt7612u_set_log_sink(capture, NULL);
	{
		char big[4096];

		memset(big, 'x', sizeof big - 1);
		big[sizeof big - 1] = '\0';
		calls = 0;
		mt_diag('I', "%s", big);
		check(calls == 1, "an oversized line still reaches the sink");
		check(strlen(last_line) < 512, "an oversized line is truncated, not overrun");
	}
	mt7612u_set_log_sink(NULL, NULL);

	fclose(stderr);
	remove(path);

	if (fails) {
		printf("log_sink: %d failure(s)\n", fails);
		return 1;
	}
	printf("log_sink: PASS\n");
	return 0;
}
