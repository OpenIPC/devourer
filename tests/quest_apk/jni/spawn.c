/* spawn.c — a minimal JNI fork()+execve() helper.
 *
 * Android's ProcessBuilder closes every fd >= 3 in the child before exec, so a
 * USB fd from UsbDeviceConnection can't reach an exec'd binary that way. Here we
 * fork from inside the app process (where the fd is valid) and execve rxdemo
 * ourselves without closing the fd — the child inherits it. Only
 * async-signal-safe calls run between fork and exec; the env array is built in
 * the parent beforehand. This is the same in-process-fd trick termux-usb and
 * PixelPilot rely on, minus the socket dance.
 */
#include <jni.h>
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

JNIEXPORT jint JNICALL
Java_org_openipc_rxq_MainActivity_nativeSpawn(JNIEnv *env, jclass cls, jint fd,
                                              jstring jexe, jstring jout,
                                              jobjectArray jenv) {
    const char *exe = (*env)->GetStringUTFChars(env, jexe, 0);
    const char *out = (*env)->GetStringUTFChars(env, jout, 0);
    char exebuf[1024], outbuf[1024];
    strncpy(exebuf, exe, sizeof(exebuf) - 1); exebuf[sizeof(exebuf) - 1] = 0;
    strncpy(outbuf, out, sizeof(outbuf) - 1); outbuf[sizeof(outbuf) - 1] = 0;
    (*env)->ReleaseStringUTFChars(env, jexe, exe);
    (*env)->ReleaseStringUTFChars(env, jout, out);

    int nenv = jenv ? (*env)->GetArrayLength(env, jenv) : 0;
    char **envp = calloc(nenv + 1, sizeof(char *));
    for (int i = 0; i < nenv; i++) {
        jstring s = (jstring)(*env)->GetObjectArrayElement(env, jenv, i);
        const char *cs = (*env)->GetStringUTFChars(env, s, 0);
        envp[i] = strdup(cs);
        (*env)->ReleaseStringUTFChars(env, s, cs);
    }
    envp[nenv] = NULL;

    char fdstr[16];
    snprintf(fdstr, sizeof(fdstr), "%d", fd);

    pid_t pid = fork();
    if (pid == 0) {
        int o = open(outbuf, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (o >= 0) { dup2(o, 1); dup2(o, 2); if (o > 2) close(o); }
        fcntl(fd, F_SETFD, 0); /* ensure the USB fd is not CLOEXEC */
        char *argv[] = { exebuf, fdstr, NULL };
        execve(exebuf, argv, envp);
        _exit(127);
    }

    for (int i = 0; i < nenv; i++) free(envp[i]);
    free(envp);
    return pid;
}

JNIEXPORT jint JNICALL
Java_org_openipc_rxq_MainActivity_nativeWait(JNIEnv *env, jclass cls, jint pid) {
    int st = 0;
    if (waitpid(pid, &st, 0) < 0) return -1;
    return WIFEXITED(st) ? WEXITSTATUS(st) : 128 + WTERMSIG(st);
}

JNIEXPORT void JNICALL
Java_org_openipc_rxq_MainActivity_nativeKill(JNIEnv *env, jclass cls, jint pid) {
    /* SIGKILL, not SIGTERM: rxdemo's SIGTERM handler runs a slow clean chip
     * de-init during which the USB device stays claimed, so the next capture's
     * openDevice/claim fails. For RX there's no TX state to unwind — kill hard
     * and reap so the device frees immediately. */
    if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
}
