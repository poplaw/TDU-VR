// TDUVR_PROFILE=<seconds>[,<delay>]: once the in-car camera has been on screen for <delay> s (default
// 10), sample the four busiest threads at 1 kHz for <seconds> and write every sample (instruction
// pointer plus the return addresses found on the stack) to tdu_prof.txt next to the log, for offline
// analysis. Off unless the variable is set.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tlhelp32.h>
#include <algorithm>
#include <cstdio>
#include <vector>
#include "vr.h"

namespace {
const int kThreads = 4, kRets = 24;
struct Sample { unsigned char th; DWORD eip; DWORD ret[kRets]; };
struct Range { DWORD lo, hi; };

double g_dur, g_delay = 10;
HANDLE g_go;
std::vector<Range> g_code;  // executable sections of every module loaded when sampling starts

bool in_code(DWORD a) {
    for (auto& r : g_code) if (a >= r.lo && a < r.hi) return true;
    return false;
}

// a is a return address if the bytes before it are a call instruction.
bool after_call(DWORD a) {
    const unsigned char* p = (const unsigned char*)a;
    __try {
        if (p[-5] == 0xE8) return true;                                       // call rel32
        if (p[-6] == 0xFF && (p[-5] == 0x15 || (p[-5] & 0xF8) == 0x90)) return true;  // call [abs] / [reg+disp32]
        if (p[-3] == 0xFF && (p[-2] & 0xF8) == 0x50) return true;           // call [reg+disp8]
        if (p[-2] == 0xFF && ((p[-1] & 0xF8) == 0xD0 || (p[-1] & 0xF8) == 0x10)) return true;  // call reg / [reg]
        if (p[-7] == 0xFF && p[-6] == 0x94) return true;                     // call [sib+disp32]
        if (p[-4] == 0xFF && p[-3] == 0x54) return true;                     // call [sib+disp8]
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    return false;
}

int exec_sections(DWORD base, Range* out, int cap) {
    int n = 0;
    __try {
        auto* dos = (IMAGE_DOS_HEADER*)base;
        auto* nt = (IMAGE_NT_HEADERS32*)(base + dos->e_lfanew);
        auto* sec = IMAGE_FIRST_SECTION(nt);
        for (int i = 0; i < nt->FileHeader.NumberOfSections && n < cap; i++)
            if (sec[i].Characteristics & IMAGE_SCN_MEM_EXECUTE)
                out[n++] = { base + sec[i].VirtualAddress, base + sec[i].VirtualAddress + sec[i].Misc.VirtualSize };
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    return n;
}

void collect_code_ranges(FILE* f) {
    HANDLE s = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, GetCurrentProcessId());
    MODULEENTRY32 me{ sizeof me };
    for (BOOL ok = Module32First(s, &me); ok; ok = Module32Next(s, &me)) {
        DWORD base = (DWORD)(UINT_PTR)me.modBaseAddr;
        fprintf(f, "M %08lx %08lx %s\n", base, me.modBaseSize, me.szExePath);
        Range r[16];
        int n = exec_sections(base, r, 16);
        g_code.insert(g_code.end(), r, r + n);
    }
    CloseHandle(s);
}

// Return addresses on a (suspended) thread's stack, innermost first.
void scan_stack(DWORD esp, DWORD top, DWORD* ret) {
    int k = 0;
    const DWORD* sp = (const DWORD*)esp;
    int n = top > esp ? (int)((top - esp) / 4) : 2048;  // up to the stack's base when known
    if (n > 4096) n = 4096;
    __try {
        for (int j = 0; j < n && k < kRets; j++)
            if (in_code(sp[j]) && after_call(sp[j])) ret[k++] = sp[j];
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

ULONGLONG thread_cpu(HANDLE h) {
    FILETIME c, e, k, u;
    if (!GetThreadTimes(h, &c, &e, &k, &u)) return 0;
    return ((ULONGLONG)k.dwHighDateTime << 32 | k.dwLowDateTime) + ((ULONGLONG)u.dwHighDateTime << 32 | u.dwLowDateTime);
}

DWORD WINAPI prof_thread(void*) {
    WaitForSingleObject(g_go, INFINITE);
    char path[MAX_PATH];
    GetModuleFileNameA(nullptr, path, MAX_PATH);
    char* slash = strrchr(path, '\\');
    strcpy_s(slash ? slash + 1 : path, MAX_PATH - (slash ? slash + 1 - path : 0), "tdu_prof.txt");
    FILE* f = nullptr;
    if (fopen_s(&f, path, "w") || !f) return 0;
    collect_code_ranges(f);

    // The busiest threads over one second.
    struct Th { DWORD id; HANDLE h; ULONGLONG t0, t1; DWORD top; };
    std::vector<Th> all;
    HANDLE s = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    THREADENTRY32 te{ sizeof te };
    for (BOOL ok = Thread32First(s, &te); ok; ok = Thread32Next(s, &te)) {
        if (te.th32OwnerProcessID != GetCurrentProcessId() || te.th32ThreadID == GetCurrentThreadId()) continue;
        HANDLE h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, te.th32ThreadID);
        if (h) all.push_back({ te.th32ThreadID, h, thread_cpu(h), 0, 0 });
    }
    CloseHandle(s);
    Sleep(1000);
    for (auto& t : all) t.t1 = thread_cpu(t.h) - t.t0;
    std::sort(all.begin(), all.end(), [](const Th& a, const Th& b) { return a.t1 > b.t1; });
    int n = (int)std::min<size_t>(kThreads, all.size());
    typedef LONG(NTAPI * QIT)(HANDLE, int, PVOID, ULONG, PULONG);
    auto qit = (QIT)GetProcAddress(GetModuleHandleA("ntdll.dll"), "NtQueryInformationThread");
    ULONGLONG cpu0[kThreads];
    for (int i = 0; i < n; i++) {
        DWORD start = 0;
        if (qit) qit(all[i].h, 9 /* ThreadQuerySetWin32StartAddress */, &start, sizeof start, nullptr);
        struct { LONG status; DWORD teb; DWORD rest[5]; } tbi{};
        if (qit && qit(all[i].h, 0 /* ThreadBasicInformation */, &tbi, sizeof tbi, nullptr) >= 0 && tbi.teb)
            all[i].top = ((const NT_TIB*)(UINT_PTR)tbi.teb)->StackBase ? (DWORD)(UINT_PTR)((const NT_TIB*)(UINT_PTR)tbi.teb)->StackBase : 0;
        fprintf(f, "T %d %lu %08lx %.1f\n", i, all[i].id, start, all[i].t1 / 1e4);  // CPU ms in that second
        cpu0[i] = thread_cpu(all[i].h);
    }

    std::vector<Sample> samples;
    samples.reserve((size_t)(g_dur * 1000 * n) + 1024);
    HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    LARGE_INTEGER due;
    due.QuadPart = -10000;  // 1 ms
    DWORD t0 = GetTickCount();
    while (GetTickCount() - t0 < g_dur * 1000 && samples.size() + n <= samples.capacity()) {
        for (int i = 0; i < n; i++) {
            // Nothing that can take a lock while the thread is suspended (it may hold the heap lock):
            // only the context and raw stack reads into memory reserved above.
            if (SuspendThread(all[i].h) == (DWORD)-1) continue;
            CONTEXT c{};
            c.ContextFlags = CONTEXT_CONTROL;
            if (GetThreadContext(all[i].h, &c)) {
                Sample sm{};
                sm.th = (unsigned char)i;
                sm.eip = c.Eip;
                scan_stack(c.Esp, all[i].top, sm.ret);
                samples.push_back(sm);
            }
            ResumeThread(all[i].h);
        }
        if (timer && SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE)) WaitForSingleObject(timer, 10);
        else Sleep(1);
    }
    double secs = (GetTickCount() - t0) / 1000.0;
    for (int i = 0; i < n; i++) fprintf(f, "C %d %.1f\n", i, (thread_cpu(all[i].h) - cpu0[i]) / 1e4 / secs / 10);  // % of a core
    for (auto& sm : samples) {
        fprintf(f, "S %d %08lx", sm.th, sm.eip);
        for (int j = 0; j < kRets && sm.ret[j]; j++) fprintf(f, " %08lx", sm.ret[j]);
        fputc('\n', f);
    }
    fclose(f);
    for (auto& t : all) CloseHandle(t.h);
    log_printf("profile: %zu samples of %d threads over %.1f s written to %s\n", samples.size(), n, secs, path);
    return 0;
}
}  // namespace

void prof_init() {
    char v[32];
    if (GetEnvironmentVariableA("TDUVR_PROFILE", v, sizeof v) == 0) return;
    g_dur = atof(v);
    if (const char* c = strchr(v, ',')) g_delay = atof(c + 1);
    if (!(g_dur > 0)) g_dur = 10;
    g_go = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    CreateThread(nullptr, 0, prof_thread, nullptr, 0, nullptr);
    log_printf("profile: %.0f s of samples, %.0f s after the in-car camera shows\n", g_dur, g_delay);
}

// Every frame: starts the sampling once the car has been on screen long enough.
void prof_frame(bool inCar) {
    static DWORD since;
    static bool fired;
    if (!g_go || fired) return;
    if (!inCar) { since = 0; return; }
    if (!since) since = GetTickCount();
    if (GetTickCount() - since >= g_delay * 1000) { SetEvent(g_go); fired = true; }
}
