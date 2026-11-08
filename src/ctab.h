#pragma once
// Reads the D3DX constant table (CTAB comment block) embedded in D3D9 shader bytecode
// and picks out the camera matrices TDU's shaders use (names verified against the exe).
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

enum MatKind {
    MK_POST_K,      // ends in *Proj after View  (WorldViewProj, ViewProj): M * K
    MK_POST_EYE,    // ends in View              (WorldView, View):         M * invEye
    MK_PROJ,        // projection alone          (gProj):                   M * inv(P) * Peye
    MK_PRE_E,       // inverse view              (InvView):                 E * M
};

struct MatSlot { int reg, count; MatKind kind; std::string name; };

struct ShaderMats {
    std::vector<MatSlot> mats;
    int pairK = -1, pairV = -1;  // X = Y*P pair (WorldViewProj+WorldView or ViewProj+View): exact P per draw
    int proj = -1;               // a bare projection matrix (gProj)
    int anyK = -1;               // any *ViewProj matrix: P's depth terms can be recovered from it
};

// Returns -1 if the name is not a camera matrix we touch.
inline int ctab_classify(const std::string& n) {
    auto has = [&](const char* s) { return n.find(s) != std::string::npos; };
    auto ends = [&](const char* s) { size_t k = strlen(s); return n.size() >= k && n.compare(n.size() - k, k, s) == 0; };
    if (has("Refl") || has("Light") || has("UV")) return -1;  // mirror/shadow/texture spaces: not the eye camera
    if (has("InvView")) return MK_PRE_E;
    if (has("ViewProj") || n == "gWVPMat") return MK_POST_K;
    if (ends("View")) return MK_POST_EYE;
    if (ends("Proj")) return MK_PROJ;
    return -1;
}

// code: shader bytecode (version token first). size_dw: bound on tokens to read.
// Calls on_ignored(name) for float matrices we skip, so the log can show what was left out.
template <class F>
inline ShaderMats ctab_parse(const uint32_t* code, size_t size_dw, F on_ignored) {
    ShaderMats out;
    struct Hdr { uint32_t Size, Creator, Version, Constants, ConstantInfo, Flags, Target; };
    struct Info { uint32_t Name; uint16_t RegisterSet, RegisterIndex, RegisterCount, Reserved; uint32_t TypeInfo, DefaultValue; };
    struct Type { uint16_t Class, Type, Rows, Columns, Elements, StructMembers; uint32_t StructMemberInfo; };

    size_t t = 1;
    for (int guard = 0; guard < 64 && t < size_dw && (code[t] & 0xFFFF) == 0xFFFE; guard++) {
        uint32_t len = code[t] >> 16;
        if (t + 1 + len > size_dw) break;
        const uint32_t* data = code + t + 1;
        if (len >= 8 && data[0] == 0x42415443u /* 'CTAB' */) {
            const uint8_t* base = (const uint8_t*)(data + 1);
            size_t bytes = (size_t)(len - 1) * 4;
            const Hdr* h = (const Hdr*)base;
            if (h->ConstantInfo + (size_t)h->Constants * sizeof(Info) > bytes) break;
            const Info* ci = (const Info*)(base + h->ConstantInfo);
            for (uint32_t i = 0; i < h->Constants; i++) {
                if (ci[i].RegisterSet != 2 /* FLOAT4 */) continue;
                if (ci[i].Name >= bytes || ci[i].TypeInfo + sizeof(Type) > bytes) continue;
                const Type* ty = (const Type*)(base + ci[i].TypeInfo);
                if (ty->Class != 2 && ty->Class != 3) continue;  // matrices only
                std::string name((const char*)base + ci[i].Name, strnlen((const char*)base + ci[i].Name, bytes - ci[i].Name));
                int k = ctab_classify(name);
                // Column-major (Class 3) is all TDU uses: register j = column j.
                // Full matrices are needed for right-multiplies; affine ones may drop column 3 only.
                bool ok = k >= 0 && ty->Class == 3 &&
                          ((k == MK_POST_K || k == MK_PROJ) ? ci[i].RegisterCount == 4 : ci[i].RegisterCount >= 3);
                if (!ok) { on_ignored(name, (int)ci[i].RegisterIndex, (int)ci[i].RegisterCount); continue; }
                out.mats.push_back(MatSlot{ (int)ci[i].RegisterIndex, (int)ci[i].RegisterCount, (MatKind)k, name });
            }
            // Pairs must share the same left part: WorldViewProj with WorldView, ViewProj with View.
            auto world = [](const std::string& n) { return n.find("World") != std::string::npos || n == "gWVPMat"; };
            for (int w = 1; w >= 0 && out.pairK < 0; w--) {
                int pk = -1, pv = -1;
                for (int i = 0; i < (int)out.mats.size(); i++) {
                    const MatSlot& m = out.mats[i];
                    if (world(m.name) != (w == 1)) continue;
                    if (m.kind == MK_POST_K && pk < 0) pk = i;
                    if (m.kind == MK_POST_EYE && pv < 0) pv = i;
                }
                if (pk >= 0 && pv >= 0) { out.pairK = pk; out.pairV = pv; }
            }
            for (int i = 0; i < (int)out.mats.size(); i++) {
                if (out.mats[i].kind == MK_POST_K && out.anyK < 0) out.anyK = i;
                if (out.mats[i].kind == MK_PROJ && out.proj < 0) out.proj = i;
            }
            break;
        }
        t += len + 1;
    }
    return out;
}
