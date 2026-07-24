package org.openipc.rxq;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/*
 * Minimal USB-host launcher for the devourer rxdemo, modelled on how
 * PixelPilot avoids the permission dialog: a USB_DEVICE_ATTACHED intent-filter
 * with a device_filter auto-grants this app permission to the matching adapter
 * and launches us with EXTRA_DEVICE set. We open the device, hand its file
 * descriptor to the native rxdemo (packaged as librxdemo.so so Android extracts
 * it executable into nativeLibraryDir), and redirect its JSONL to
 * getExternalFilesDir() where adb can pull it. Everything is driven headlessly
 * via `am start` extras (rxmode/channel/tag/...); no in-headset interaction.
 */
public class MainActivity extends Activity {
    static final String TAG = "RXQ";
    static final String ACTION_PERM = "org.openipc.rxq.PERM";
    static final int VID = 0x0bda, PID = 0x8812;

    // The running rxdemo child pid + reaper thread, kept across launches so a new
    // run (or action=stop) can reap the previous one. sConn is held open for the
    // child's lifetime so the USB fd stays valid.
    static int sPid;
    static Thread sWaiter;
    static UsbDeviceConnection sConn;

    static { System.loadLibrary("spawn"); }

    // JNI fork()+execve() helper (spawn.c) — ProcessBuilder can't hand a USB fd
    // to a child (it closes all fds >= 3 before exec), so we fork in-process
    // where the fd is valid and exec rxdemo ourselves without closing it.
    static native int nativeSpawn(int fd, String exe, String out, String[] env);
    static native int nativeWait(int pid);
    static native void nativeKill(int pid);

    @Override protected void onCreate(Bundle b) {
        super.onCreate(b);
        handle(getIntent());
    }

    @Override protected void onNewIntent(Intent i) {
        super.onNewIntent(i);
        setIntent(i);
        handle(i);
    }

    void handle(Intent intent) {
        if (intent != null && "stop".equals(intent.getStringExtra("action"))) {
            killChild();
            writeStatus("stopped");
            Log.i(TAG, "stopped on request");
            return;
        }
        UsbManager um = (UsbManager) getSystemService(Context.USB_SERVICE);
        UsbDevice dev = null;
        if (intent != null)
            dev = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
        if (dev == null) dev = findDevice(um);
        if (dev == null) {
            Log.e(TAG, "no matching USB device (" + Integer.toHexString(VID)
                    + ":" + Integer.toHexString(PID) + ")");
            writeStatus("no-device");
            return;
        }
        if (um.hasPermission(dev)) {
            launch(um, dev, intent);
        } else {
            // Fallback for the already-attached case: request permission. With
            // the intent-filter present this is normally auto-granted on
            // attach, so this path is a safety net.
            Log.w(TAG, "no permission yet, requesting");
            // targetSdk 28 is exempt from the API-31 mutable-PendingIntent
            // requirement, so FLAG_UPDATE_CURRENT compiles against older jars.
            PendingIntent pi = PendingIntent.getBroadcast(this, 0,
                    new Intent(ACTION_PERM), PendingIntent.FLAG_UPDATE_CURRENT);
            registerReceiver(new BroadcastReceiver() {
                @Override public void onReceive(Context c, Intent i) {
                    UsbDevice d = i.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    boolean ok = i.getBooleanExtra(
                            UsbManager.EXTRA_PERMISSION_GRANTED, false);
                    Log.i(TAG, "perm result granted=" + ok);
                    if (ok && d != null) launch(um, d, getIntent());
                    try { c.unregisterReceiver(this); } catch (Exception e) {}
                }
            }, new IntentFilter(ACTION_PERM));
            um.requestPermission(dev, pi);
        }
    }

    UsbDevice findDevice(UsbManager um) {
        HashMap<String, UsbDevice> m = um.getDeviceList();
        for (UsbDevice d : m.values())
            if (d.getVendorId() == VID && d.getProductId() == PID) return d;
        return null;
    }

    void launch(UsbManager um, UsbDevice dev, Intent intent) {
        killChild();
        UsbDeviceConnection conn = um.openDevice(dev);
        if (conn == null) {
            Log.e(TAG, "openDevice returned null");
            writeStatus("open-failed");
            return;
        }
        // fork()+execve() in-process (spawn.c): the fd from getFileDescriptor()
        // is valid here and the native helper does not close it, so the child
        // inherits it directly. Keep sConn open for the child's lifetime.
        int fd = conn.getFileDescriptor();
        sConn = conn;
        String tag = ext(intent, "tag", "run");
        File out = new File(getExternalFilesDir(null), "rxq_" + tag + ".jsonl");
        String nlib = getApplicationInfo().nativeLibraryDir;

        Map<String, String> e = new HashMap<>();
        e.put("LD_LIBRARY_PATH", nlib);
        e.put("DEVOURER_EVENTS", "stdout");
        e.put("DEVOURER_LOG_LEVEL", ext(intent, "loglevel", "info"));
        e.put("DEVOURER_CHANNEL", ext(intent, "channel", "36"));
        e.put("DEVOURER_RX_MODE", ext(intent, "rxmode", "async"));
        e.put("DEVOURER_RX_URBS", ext(intent, "urbs", "4"));
        // rx.seq is the ground-truth per-frame delivery sequence (pctr counter
        // the txdemo QoS-Data flood stamps); always on for this study.
        e.put("DEVOURER_RX_PCTR", ext(intent, "pctr", "1"));
        putIf(e, "DEVOURER_RX_POOL_SPARE", ext(intent, "poolspare", null));
        putIf(e, "DEVOURER_RX_RING_MS", ext(intent, "ringms", "100"));
        putIf(e, "DEVOURER_RX_SINK_SPIN_US", ext(intent, "sinkspin", null));
        putIf(e, "DEVOURER_RX_KEEP_CORRUPTED", ext(intent, "keepcorrupt", null));
        putIf(e, "DEVOURER_BW", ext(intent, "bw", null));
        ArrayList<String> envList = new ArrayList<>();
        for (Map.Entry<String, String> en : e.entrySet())
            envList.add(en.getKey() + "=" + en.getValue());
        String[] envArr = envList.toArray(new String[0]);

        Log.i(TAG, "spawn fd=" + fd + " mode=" + e.get("DEVOURER_RX_MODE")
                + " ch=" + e.get("DEVOURER_CHANNEL") + " -> " + out);
        final int pid = nativeSpawn(fd, nlib + "/librxdemo.so",
                out.getAbsolutePath(), envArr);
        sPid = pid;
        if (pid <= 0) {
            writeStatus("spawn-failed pid=" + pid);
            return;
        }
        writeStatus("running tag=" + tag + " pid=" + pid + " fd=" + fd
                + " mode=" + e.get("DEVOURER_RX_MODE"));
        sWaiter = new Thread(new Runnable() {
            @Override public void run() {
                int rc = nativeWait(pid);
                Log.i(TAG, "rxdemo pid=" + pid + " exit " + rc);
                writeStatus("exited rc=" + rc + " pid=" + pid);
            }
        });
        sWaiter.start();
    }

    static void killChild() {
        if (sPid > 0) { nativeKill(sPid); }
        sPid = 0;
        // Release the USB device so the next capture's openDevice/claim is clean.
        if (sConn != null) { try { sConn.close(); } catch (Exception e) {} sConn = null; }
    }

    void putIf(Map<String, String> m, String k, String v) {
        if (v != null && !v.isEmpty()) m.put(k, v);
    }

    static String ext(Intent i, String k, String def) {
        if (i == null) return def;
        String v = i.getStringExtra(k);
        return v != null ? v : def;
    }

    void writeStatus(String s) {
        try {
            File f = new File(getExternalFilesDir(null), "status.txt");
            java.io.FileWriter w = new java.io.FileWriter(f, false);
            w.write(s + "\n");
            w.close();
        } catch (Exception e) { Log.e(TAG, "status write", e); }
    }
}
