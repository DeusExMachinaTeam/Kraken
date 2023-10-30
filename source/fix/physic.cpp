#include "float.h"

#include <smmintrin.h>

#include "routines.hpp"
#include "fix/physic.hpp"

namespace kraken::fix::physic {
    #define RIGHT_SHIFT(T, base, offset) ((T*)(reinterpret_cast<char*>(base) + offset))

    struct dxQuickStepParameters;
    struct dxContactParameters;
    struct dxWorld;
    struct dObject;
    struct dxAutoDisable;
    struct dxDamping;
    struct dMass;
    struct dxGeom;
    struct dxBody;
    struct dxJointInfo1;
    struct dxJointInfo2;
    struct dxJointNode;
    struct dJointFeedback;
    struct dxJointVtable;
    struct dxJointBreakInfo;
    struct dxJoint;
    struct dxJointGroup;
    struct dObStack;
    struct dObStackArena;
    typedef dxJointGroup* dJointGroupID;

    #define dDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])

    inline float dDOT   (const float *a, const float *b) { return dDOTpq(a,b,1,1); }
    inline float dDOT13 (const float *a, const float *b) { return dDOTpq(a,b,1,3); }
    inline float dDOT31 (const float *a, const float *b) { return dDOTpq(a,b,3,1); }
    inline float dDOT33 (const float *a, const float *b) { return dDOTpq(a,b,3,3); }
    inline float dDOT14 (const float *a, const float *b) { return dDOTpq(a,b,1,4); }
    inline float dDOT41 (const float *a, const float *b) { return dDOTpq(a,b,4,1); }
    inline float dDOT44 (const float *a, const float *b) { return dDOTpq(a,b,4,4); }

    #define dPAD(a) (((a) > 1) ? ((((a)-1)|3)+1) : (a))

    #define dCROSS(a,op,b,c) \
        (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
        (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
        (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);

    enum {
        // if this flag is set, the joint wil break
        dJOINT_BROKEN =             0x0001,
        // if this flag is set, the joint wil be deleted when it breaks
        dJOINT_DELETE_ON_BREAK =    0x0002,
        // if this flag is set, the joint can break at a certain force on body 1
        dJOINT_BREAK_AT_B1_FORCE =  0x0004,
        // if this flag is set, the joint can break at a certain torque on body 1
        dJOINT_BREAK_AT_B1_TORQUE = 0x0008,
        // if this flag is set, the joint can break at a certain force on body 2
        dJOINT_BREAK_AT_B2_FORCE =  0x0010,
        // if this flag is set, the joint can break at a certain torque on body 2
        dJOINT_BREAK_AT_B2_TORQUE = 0x0020
    };

    #define dMULTIPLYOP0_331(A,op,B,C) \
        (A)[0] op dDOT((B),(C)); \
        (A)[1] op dDOT((B+4),(C)); \
        (A)[2] op dDOT((B+8),(C));
    #define dMULTIPLYOP1_331(A,op,B,C) \
        (A)[0] op dDOT41((B),(C)); \
        (A)[1] op dDOT41((B+1),(C)); \
        (A)[2] op dDOT41((B+2),(C));
    #define dMULTIPLYOP0_133(A,op,B,C) \
        (A)[0] op dDOT14((B),(C)); \
        (A)[1] op dDOT14((B),(C+1)); \
        (A)[2] op dDOT14((B),(C+2));
    #define dMULTIPLYOP0_333(A,op,B,C) \
        (A)[0] op dDOT14((B),(C)); \
        (A)[1] op dDOT14((B),(C+1)); \
        (A)[2] op dDOT14((B),(C+2)); \
        (A)[4] op dDOT14((B+4),(C)); \
        (A)[5] op dDOT14((B+4),(C+1)); \
        (A)[6] op dDOT14((B+4),(C+2)); \
        (A)[8] op dDOT14((B+8),(C)); \
        (A)[9] op dDOT14((B+8),(C+1)); \
        (A)[10] op dDOT14((B+8),(C+2));
    #define dMULTIPLYOP1_333(A,op,B,C) \
        (A)[0] op dDOT44((B),(C)); \
        (A)[1] op dDOT44((B),(C+1)); \
        (A)[2] op dDOT44((B),(C+2)); \
        (A)[4] op dDOT44((B+1),(C)); \
        (A)[5] op dDOT44((B+1),(C+1)); \
        (A)[6] op dDOT44((B+1),(C+2)); \
        (A)[8] op dDOT44((B+2),(C)); \
        (A)[9] op dDOT44((B+2),(C+1)); \
        (A)[10] op dDOT44((B+2),(C+2));
    #define dMULTIPLYOP2_333(A,op,B,C) \
        (A)[0] op dDOT((B),(C)); \
        (A)[1] op dDOT((B),(C+4)); \
        (A)[2] op dDOT((B),(C+8)); \
        (A)[4] op dDOT((B+4),(C)); \
        (A)[5] op dDOT((B+4),(C+4)); \
        (A)[6] op dDOT((B+4),(C+8)); \
        (A)[8] op dDOT((B+8),(C)); \
        (A)[9] op dDOT((B+8),(C+4)); \
        (A)[10] op dDOT((B+8),(C+8));

    #define DECL template <class TA, class TB, class TC> inline void

    DECL dMULTIPLY0_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_331(A,=,B,C) }
    DECL dMULTIPLY1_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_331(A,=,B,C) }
    DECL dMULTIPLY0_133(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_133(A,=,B,C) }
    DECL dMULTIPLY0_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_333(A,=,B,C) }
    DECL dMULTIPLY1_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_333(A,=,B,C) }
    DECL dMULTIPLY2_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP2_333(A,=,B,C) }

    DECL dMULTIPLYADD0_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_331(A,+=,B,C) }
    DECL dMULTIPLYADD1_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_331(A,+=,B,C) }
    DECL dMULTIPLYADD0_133(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_133(A,+=,B,C) }
    DECL dMULTIPLYADD0_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_333(A,+=,B,C) }
    DECL dMULTIPLYADD1_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_333(A,+=,B,C) }
    DECL dMULTIPLYADD2_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP2_333(A,+=,B,C) }

    typedef void (__fastcall* PFNdSolveLCP) (int n, float *A, float *x, float *b, float *w, int nub, float *lo, float *hi, int *findex);
    PFNdSolveLCP dSolveLCP = (PFNdSolveLCP) 0x00956450;

    typedef void (__fastcall* PFNdxStepBody) (dxBody *b, float h);
    PFNdxStepBody dxStepBody = (PFNdxStepBody) 0x008FC8F0;

    void MultiplyAdd1_8q1(float *A, float *B, float *C, int q) {
        int k;
        float sum;
        sum = 0;
        for (k=0; k<q; k++) sum += B[k*8] * C[k];
        A[0] += sum;
        sum = 0;
        for (k=0; k<q; k++) sum += B[1+k*8] * C[k];
        A[1] += sum;
        sum = 0;
        for (k=0; k<q; k++) sum += B[2+k*8] * C[k];
        A[2] += sum;
        sum = 0;
        for (k=0; k<q; k++) sum += B[4+k*8] * C[k];
        A[4] += sum;
        sum = 0;
        for (k=0; k<q; k++) sum += B[5+k*8] * C[k];
        A[5] += sum;
        sum = 0;
        for (k=0; k<q; k++) sum += B[6+k*8] * C[k];
        A[6] += sum;
    }

    void MultiplyAdd2_p8r(float *A, float *B, float *C, int p, int r, int Askip) {
        int i,j;
        float sum,*bb,*cc;
        bb = B;
        for (i=p; i; i--) {
            cc = C;
            for (j=r; j; j--) {
                sum = bb[0]*cc[0];
                sum += bb[1]*cc[1];
                sum += bb[2]*cc[2];
                sum += bb[4]*cc[4];
                sum += bb[5]*cc[5];
                sum += bb[6]*cc[6];
                *(A++) += sum;
                cc += 8;
            }
            A += Askip - r;
            bb += 8;
        }
    }

    void Multiply2_p8r(float *A, float *B, float *C, int p, int r, int Askip) {
        int i,j;
        float sum,*bb,*cc;
        bb = B;
        for (i=p; i; i--) {
            cc = C;
            for (j=r; j; j--) {
                sum = bb[0]*cc[0];
                sum += bb[1]*cc[1];
                sum += bb[2]*cc[2];
                sum += bb[4]*cc[4];
                sum += bb[5]*cc[5];
                sum += bb[6]*cc[6];
                *(A++) = sum; 
                cc += 8;
            }
            A += Askip - r;
            bb += 8;
        }
    }

    void Multiply0_p81(float *A, float *B, float *C, int p) {
        int i;
        float sum;
        for (i=p; i; i--) {
            sum =  B[0]*C[0];
            sum += B[1]*C[1];
            sum += B[2]*C[2];
            sum += B[4]*C[4];
            sum += B[5]*C[5];
            sum += B[6]*C[6];
            *(A++) = sum;
            B += 8;
        }
    }

    void MultiplyAdd0_p81(float *A, float *B, float *C, int p) {
        int i;
        float sum;
        for (i=p; i; i--) {
            sum =  B[0]*C[0];
            sum += B[1]*C[1];
            sum += B[2]*C[2];
            sum += B[4]*C[4];
            sum += B[5]*C[5];
            sum += B[6]*C[6];
            *(A++) += sum;
            B += 8;
        }
    }

    static void Multiply1_8q1(float *A, float *B, float *C, int q) {
        int k;
        float sum;
        sum = 0;
        for (k=0; k<q; k++)
             sum += B[k*8] * C[k];
        A[0] = sum;
        sum = 0;
        for (k=0; k<q; k++)
             sum += B[1+k*8] * C[k];
        A[1] = sum;
        sum = 0;
        for (k=0; k<q; k++)
             sum += B[2+k*8] * C[k];
        A[2] = sum;
        sum = 0;
        for (k=0; k<q; k++)
             sum += B[4+k*8] * C[k];
        A[4] = sum;
        sum = 0;
        for (k=0; k<q; k++)
             sum += B[5+k*8] * C[k];
        A[5] = sum;
        sum = 0;
        for (k=0; k<q; k++)
             sum += B[6+k*8] * C[k];
        A[6] = sum;
    }

    template <typename T>
    struct HeapArray
    {
        T *ptr          = nullptr;
        DWORD* checksum = nullptr;
        size_t size     = 0;
        size_t count    = 0;

        inline HeapArray(size_t count) {
            this->count = count;
            this->size = sizeof(T) * count + sizeof(DWORD);
            if (this->size & 0xF)
                this->size = ((this->size >> 4) << 4) + 0x10;

            this->ptr = (T *)malloc(this->size);

            this->checksum = RIGHT_SHIFT(DWORD, this->ptr, this->size - sizeof(DWORD));
            *this->checksum = 0xDEADBEEF;
        };
        inline ~HeapArray() {
            if (*this->checksum != 0xDEADBEEF)
                DebugBreak();
            free(this->ptr);
        };
        inline void SetZero() {
            memset(this->ptr, 0, this->size - sizeof(DWORD));
        };
        inline void SetValue(T value) {
            for (size_t i = 0; i < this->count; i++)
                this->ptr[i] = value;
        };
    };

    struct dObStackArena {
        dObStackArena* next;
        int            used;
    };

    struct dObStack {
        dObStackArena* first;
        dObStackArena* last;
        dObStackArena* current_arena;
        int              current_ofs;
    };

    struct dxJointGroup {
        int num;
        dObStack stack;
    };

    struct dxJointGroupID {

    };

    struct dxQuickStepParameters {
        int num_iterations;
        float w;
    };

    struct dxContactParameters {
        float max_vel;
        float min_depth;
    };

    struct dxAutoDisable {
        float linear_threshold;
        float angular_threshold;
        float idle_time;
        int   idle_steps;
    };

    struct dxDamping {
        float m_linearDamping;
        float m_angularDamping;
    };

    struct dxWorld {
        dxBody*               m_firstEnabledBody;
        dxBody*               m_firstDisabledBody;
        dxJoint*              firstjoint;
        int                   nb;
        int                   nj;
        float                 gravity[4];
        float                 global_erp;
        float                 global_cfm;
        dxAutoDisable         adis;
        dxDamping             m_damping;
        int                   adis_flag;
        bool                  m_dampingFlag;
        dxQuickStepParameters qs;
        dxContactParameters   contactp;
    };

    struct dObject {
        dxWorld*  world;
        dObject*  next;
        dObject** tome;
        void*     userdata;
        int32_t   tag;
    };

    struct dMass {
        float mass;
        float c[4];
        float I[12];
    };

    struct dxGeom {};

    struct dxBody: dObject {
        dxJointNode*  firstjoint;
        int32_t       flags;
        dxGeom*       geom;
        dMass         mass;
        float         invI[12];
        float         invMass;
        float         pos[4];
        float         q[4];
        float         R[12];
        float         lvel[4];
        float         avel[4];
        float         facc[4];
        float         tacc[4];
        float         finite_rot_axis[4];
        dxAutoDisable adis;
        dxDamping     m_damping;
        float         adis_timeleft;
        int           adis_stepsleft;
        void (__fastcall* m_movedCallback)(dxBody *);
        void (__fastcall* m_changeEnabledStateCallback)(dxBody *);
    };

    struct dxJointInfo1 {
        int32_t m;
        int32_t nub;
    };

    struct dxJointInfo2 {
        float   fps;
        float   erp;
        float*  J1l;
        float*  J1a;
        float*  J2l;
        float*  J2a;
        int32_t rowskip;
        float*  c;
        float*  cfm;
        float*  lo;
        float*  hi;
        int*    findex;
    };

    struct dxJointNode {
        dxJoint*     joint;
        dxBody*      body;
        dxJointNode* next;
    };

    struct dJointFeedback {
        float f1[4];
        float t1[4];
        float f2[4];
        float t2[4];
    };

    struct dxJointVtable {
        int32_t size;
        void (__fastcall *init    )(dxJoint *);
        void (__fastcall *getInfo1)(dxJoint *, dxJointInfo1 *);
        void (__fastcall *getInfo2)(dxJoint *, dxJointInfo2 *);
        int32_t typenum;
    };

    struct dxJointBreakInfo {
        int32_t flags;
        float   b1MaxF[3];
        float   b1MaxT[3];
        float   b2MaxF[3];
        float   b2MaxT[3];
        void (__fastcall *callback)(dxJoint *);
    };

    struct dxJoint : dObject {
        dxJointVtable*    vtable;
        int32_t           flags;
        dxJointNode       node[2];
        dJointFeedback*   feedback;
        float             lambda[6];
        dxJointBreakInfo* breakInfo;
    };

    enum {
        dxBodyFlagFiniteRotation = 1,        // use finite rotations
        dxBodyFlagFiniteRotationAxis = 2,    // use finite rotations only along axis
        dxBodyDisabled = 4,            // body is disabled
        dxBodyNoGravity = 8,            // body is not influenced by gravity
        dxBodyAutoDisable = 16        // enable auto-disable on body
    };

    void __fastcall dInternalStepIsland_x2(dxWorld* world, dxBody*const* body, int nb, dxJoint*const* _joint, int nj, float stepsize) {
        int i, j, k;
        float stepsize1 = 1.0 / stepsize;

        for (i = 0; i < nb; i++)
            body[i]->tag = i;

        HeapArray<float>    I     (3*nb*4);
        HeapArray<float>    invI  (3*nb*4);
        HeapArray<dxJoint*> joint (nj);
        memcpy(joint.ptr, _joint, nj*sizeof(dxJoint*));

        for (i=0; i<nb; i++) {
            float tmp[12];
            dMULTIPLY2_333(tmp,             body[i]->mass.I, body[i]->R);
            dMULTIPLY0_333(I.ptr + i * 12,  body[i]->R,      tmp);
            dMULTIPLY2_333(tmp,             body[i]->invI,   body[i]->R);
            dMULTIPLY0_333(invI.ptr + i*12, body[i]->R,      tmp);
            dMULTIPLY0_331(tmp,             I.ptr + i * 12,  body[i]->avel);
            dCROSS(body[i]->tacc,-=,body[i]->avel,tmp);
        }

        for (i=0; i<nb; i++) {
            if ((body[i]->flags & dxBodyNoGravity)==0) {
                body[i]->facc[0] += body[i]->mass.mass * world->gravity[0];
                body[i]->facc[1] += body[i]->mass.mass * world->gravity[1];
                body[i]->facc[2] += body[i]->mass.mass * world->gravity[2];
            }
        }

        int m = 0;
        HeapArray<dxJointInfo1> info (nj);
        HeapArray<int> ofs (nj);
        for (i=0, j=0; j<nj; j++) {    // i=dest, j=src
            joint.ptr[j]->vtable->getInfo1(joint.ptr[j], info.ptr + i);
            if (info.ptr[i].m > 0) {
                joint.ptr[i] = joint.ptr[j];
                joint.ptr[i]->tag = i;
                i++;
            }
            else {
                joint.ptr[j]->tag = -1;
            }
        }
        nj = i;

        for (i=0; i<nj; i++)
            if (info.ptr[i].nub == info.ptr[i].m) {
                ofs.ptr[i] = m;
                m += info.ptr[i].m;
            }
        int nub = m;

        for (i=0; i<nj; i++)
            if (info.ptr[i].nub > 0 && info.ptr[i].nub < info.ptr[i].m) {
                ofs.ptr[i] = m;
                m += info.ptr[i].m;
            }

        for (i=0; i<nj; i++) if (info.ptr[i].nub == 0) {
            ofs.ptr[i] = m;
            m += info.ptr[i].m;
        }

        HeapArray<float> cforce (nb * 8);
        cforce.SetZero();

        if (m > 0) {
            HeapArray<float> c (m);
            c.SetZero();

            HeapArray<float> cfm (m);
            cfm.SetValue(world->global_cfm);

            HeapArray<float> lo (m);
            lo.SetValue(-FLT_MAX);

            HeapArray<float> hi (m);
            hi.SetValue(FLT_MAX);

            HeapArray<int32_t> findex (m);
            findex.SetValue(-1);

            HeapArray<float> J (2 * m * 8);
            J.SetZero();

            dxJointInfo2 Jinfo;

            Jinfo.rowskip = 8;
            Jinfo.fps     = stepsize1;
            Jinfo.erp     = world->global_erp;
            for (i=0; i<nj; i++) {
                Jinfo.J1l    = J.ptr + 2 * 8 * ofs.ptr[i];
                Jinfo.J1a    = Jinfo.J1l + 4;
                Jinfo.J2l    = Jinfo.J1l + 8 * info.ptr[i].m;
                Jinfo.J2a    = Jinfo.J2l + 4;
                Jinfo.c      = c.ptr + ofs.ptr[i];
                Jinfo.cfm    = cfm.ptr + ofs.ptr[i];
                Jinfo.lo     = lo.ptr + ofs.ptr[i];
                Jinfo.hi     = hi.ptr + ofs.ptr[i];
                Jinfo.findex = findex.ptr + ofs.ptr[i];

                joint.ptr[i]->vtable->getInfo2(joint.ptr[i], &Jinfo);

                for (j=0; j<info.ptr[i].m; j++) {
                    if (findex.ptr[ofs.ptr[i] + j] >= 0) findex.ptr[ofs.ptr[i] + j] += ofs.ptr[i];
                }
            }

            HeapArray<float> JinvM (2*m*8);
            JinvM.SetZero();

            for (i=0; i<nj; i++) {
                int b = joint.ptr[i]->node[0].body->tag;
                float body_invMass = body[b]->invMass;
                float *body_invI   = invI.ptr + b * 12;
                float *Jsrc        = J.ptr + 2 * 8 * ofs.ptr[i];
                float *Jdst        = JinvM.ptr + 2 * 8 * ofs.ptr[i];

                for (j=info.ptr[i].m-1; j>=0; j--) {
                    for (k=0; k<3; k++)
                        Jdst[k] = Jsrc[k] * body_invMass;

                    dMULTIPLY0_133 (Jdst+4,Jsrc+4,body_invI);
                    Jsrc += 8;
                    Jdst += 8;
                }

                if (joint.ptr[i]->node[1].body) {
                    b = joint.ptr[i]->node[1].body->tag;
                    body_invMass = body[b]->invMass;
                    body_invI    = invI.ptr + b * 12;

                    for (j=info.ptr[i].m-1; j>=0; j--) {
                        for (k=0; k<3; k++)
                            Jdst[k] = Jsrc[k] * body_invMass;

                        dMULTIPLY0_133 (Jdst+4,Jsrc+4,body_invI);

                        Jsrc += 8;
                        Jdst += 8;
                    }
                }
            }

            int mskip = dPAD(m);

            HeapArray<float> A (m * mskip);
            A.SetZero();

            for (i=0; i<nb; i++) {
                for (dxJointNode *n1=body[i]->firstjoint; n1; n1=n1->next) {
                    for (dxJointNode *n2=n1->next; n2; n2=n2->next) {
                        int j1 = n1->joint->tag;
                        int j2 = n2->joint->tag;

                        if (ofs.ptr[j1] < ofs.ptr[j2]) {
                            int tmp = j1;
                            j1 = j2;
                            j2 = tmp;
                        }

                        if (j1==-1 || j2==-1)
                            continue;

                        int jb1 = (joint.ptr[j1]->node[1].body == body[i]);
                        int jb2 = (joint.ptr[j2]->node[1].body == body[i]);

                        MultiplyAdd2_p8r(
                            A.ptr     + ofs.ptr[j1] * mskip + ofs.ptr[j2],
                            JinvM.ptr + 2 * 8 * ofs.ptr[j1] + jb1 * 8 * info.ptr[j1].m,
                            J.ptr     + 2 * 8 * ofs.ptr[j2] + jb2 * 8 * info.ptr[j2].m,
                            info.ptr[j1].m,
                            info.ptr[j2].m,
                            mskip
                        );
                    }
                }
            }

            for (i=0; i<nj; i++) {
                Multiply2_p8r(
                    A.ptr     + ofs.ptr[i] * (mskip+1),
                    JinvM.ptr + 2 * 8 * ofs.ptr[i],
                    J.ptr     + 2 * 8 * ofs.ptr[i],
                    info.ptr[i].m,
                    info.ptr[i].m,
                    mskip
                );

                if (joint.ptr[i]->node[1].body) {
                    MultiplyAdd2_p8r(
                        A.ptr     + ofs.ptr[i] * (mskip+1),
                        JinvM.ptr + 2 * 8 * ofs.ptr[i] + 8 * info.ptr[i].m,
                        J.ptr     + 2 * 8 * ofs.ptr[i] + 8 * info.ptr[i].m,
                        info.ptr[i].m,
                        info.ptr[i].m,
                        mskip
                    );
                }
            }

            // TODO: [INVESTIAGE] Unknown for cycle
            // Need resolve what cycle here in source
            // see dInternalStepIsland_x2:899

            for (i=0; i<m; i++)
                A.ptr[i * mskip + i] += cfm.ptr[i] * stepsize1;


            HeapArray<float> tmp1 (nb * 8);
            for (i=0; i<nb; i++) {
                float body_invMass = body[i]->invMass;
                float *body_invI   = invI.ptr + i * 12;

                for (j=0; j<3; j++)
                    tmp1.ptr[i*8+j] = body[i]->facc[j] * body_invMass + body[i]->lvel[j] * stepsize1;
                
                dMULTIPLY0_331(tmp1.ptr + i * 8 + 4, body_invI ,body[i]->tacc);
                
                for (j=0; j<3; j++)
                    tmp1.ptr[i * 8 + 4 + j] += body[i]->avel[j] * stepsize1;
            }

            HeapArray<float> rhs (m);
            for (i=0; i<nj; i++) {
                float *JJ = J.ptr + 2 * 8 * ofs.ptr[i];
                Multiply0_p81 (
                    rhs.ptr + ofs.ptr[i],
                    JJ,
                    tmp1.ptr + 8 * joint.ptr[i]->node[0].body->tag,
                    info.ptr[i].m
                );

                if (joint.ptr[i]->node[1].body) {
                    MultiplyAdd0_p81 (
                        rhs.ptr + ofs.ptr[i],
                        JJ + 8 * info.ptr[i].m,
                        tmp1.ptr + 8 * joint.ptr[i]->node[1].body->tag,
                        info.ptr[i].m
                    );
                }
            }

            for (i=0; i<m; i++)
                rhs.ptr[i] = c.ptr[i]*stepsize1 - rhs.ptr[i];

            HeapArray<float> lambda (m);
            HeapArray<float> residual (m);

            dSolveLCP(
                m,
                A.ptr,
                lambda.ptr,
                rhs.ptr,
                residual.ptr,
                nub,
                lo.ptr,
                hi.ptr,
                findex.ptr
            );

            for (i=0; i<nj; i++) {
                float *JJ = J.ptr + 2 * 8 * ofs.ptr[i];

                dxBody*           b1  = joint.ptr[i]->node[0].body;
                dxBody*           b2  = joint.ptr[i]->node[1].body;
                dJointFeedback*   fb  = joint.ptr[i]->feedback;
                dxJointBreakInfo* jBI = joint.ptr[i]->breakInfo;

                if (jBI || fb) {
                    dJointFeedback temp_fb;

                    float data1[8];
                    float data2[8];

                    Multiply1_8q1(data1, JJ, lambda.ptr + ofs.ptr[i], info.ptr[i].m);

                    float* cf1 = cforce.ptr + 8 * b1->tag;

                    cf1[0] += (temp_fb.f1[0] = data1[0]);
                    cf1[1] += (temp_fb.f1[1] = data1[1]);
                    cf1[2] += (temp_fb.f1[2] = data1[2]);
                    cf1[4] += (temp_fb.t1[0] = data1[4]);
                    cf1[5] += (temp_fb.t1[1] = data1[5]);
                    cf1[6] += (temp_fb.t1[2] = data1[6]);

                    if (b2) {
                        Multiply1_8q1 (data2, JJ + 8 * info.ptr[i].m, lambda.ptr + ofs.ptr[i], info.ptr[i].m);

                        float *cf2 = cforce.ptr + 8 * b2->tag;

                        cf2[0] += (temp_fb.f2[0] = data2[0]);
                        cf2[1] += (temp_fb.f2[1] = data2[1]);
                        cf2[2] += (temp_fb.f2[2] = data2[2]);
                        cf2[4] += (temp_fb.t2[0] = data2[4]);
                        cf2[5] += (temp_fb.t2[1] = data2[5]);
                        cf2[6] += (temp_fb.t2[2] = data2[6]);
                    }

                    if (fb) {
                        fb->f1[0] = temp_fb.f1[0];
                        fb->f1[1] = temp_fb.f1[1];
                        fb->f1[2] = temp_fb.f1[2];
                        fb->t1[0] = temp_fb.t1[0];
                        fb->t1[1] = temp_fb.t1[1];
                        fb->t1[2] = temp_fb.t1[2];

                        if (b2) {
                            fb->f2[0] = temp_fb.f2[0];
                            fb->f2[1] = temp_fb.f2[1];
                            fb->f2[2] = temp_fb.f2[2];
                            fb->t2[0] = temp_fb.t2[0];
                            fb->t2[1] = temp_fb.t2[1];
                            fb->t2[2] = temp_fb.t2[2];
                        }
                    }

                    if (jBI) {
                        float relCF1[3];
                        float relCT1[3];

                        dMULTIPLY1_331(&relCF1[0],b1->R,&temp_fb.f1[0]);
                        dMULTIPLY1_331(&relCT1[0],b1->R,&temp_fb.t1[0]);

                        if (jBI->flags & dJOINT_BREAK_AT_B1_FORCE) {
                            for (int i = 0; i < 3; i++) {
                                if (relCF1[i] > jBI->b1MaxF[i]) {
                                    jBI->flags |= dJOINT_BROKEN;
                                    goto doneCheckingBreaks;
                                }
                            }
                        }

                        if (jBI->flags & dJOINT_BREAK_AT_B1_TORQUE) {
                            for (int i = 0; i < 3; i++) {
                                if (relCT1[i] > jBI->b1MaxT[i]) {
                                    jBI->flags |= dJOINT_BROKEN;
                                    goto doneCheckingBreaks;
                                }
                            }
                        }

                        if (b2) {
                            float relCF2[3];
                            float relCT2[3];

                            dMULTIPLY1_331(&relCF2[0],b2->R,&temp_fb.f2[0]);
                            dMULTIPLY1_331(&relCT2[0],b2->R,&temp_fb.t2[0]);

                            if (jBI->flags & dJOINT_BREAK_AT_B2_FORCE) {
                                for (int i = 0; i < 3; i++) {
                                    if (relCF2[i] > jBI->b2MaxF[i]) {
                                        jBI->flags |= dJOINT_BROKEN;
                                        goto doneCheckingBreaks;
                                    }
                                }
                            }

                            if (jBI->flags & dJOINT_BREAK_AT_B2_TORQUE) {
                                for (int i = 0; i < 3; i++) {
                                    if (relCT2[i] > jBI->b2MaxT[i]) {
                                        jBI->flags |= dJOINT_BROKEN;
                                        goto doneCheckingBreaks;
                                    }
                                }
                            }
                        }

                        doneCheckingBreaks:;
                    }
                }
                else {
                    MultiplyAdd1_8q1(cforce.ptr + 8 * b1->tag, JJ, lambda.ptr + ofs.ptr[i], info.ptr[i].m);

                    if (b2) {
                        MultiplyAdd1_8q1 (
                            cforce.ptr + 8 * b2->tag,
                            JJ + 8 * info.ptr[i].m,
                            lambda.ptr + ofs.ptr[i],
                            info.ptr[i].m
                        );
                    }
                }
            }
        }

        for (i=0; i<nb; i++) {
            for (j=0; j<3; j++) cforce.ptr[i*8+j] += body[i]->facc[j];
            for (j=0; j<3; j++) cforce.ptr[i*8+4+j] += body[i]->tacc[j];
        }

        for (i=0; i < nb*8; i++)
            cforce.ptr[i] *= stepsize;

        for (i=0; i<nb; i++) {
            float body_invMass = body[i]->invMass;
            float *body_invI = invI.ptr + i * 12;
            for (j=0; j<3; j++) body[i]->lvel[j] += body_invMass * cforce.ptr[i*8+j];
            dMULTIPLYADD0_331 (body[i]->avel,body_invI,cforce.ptr + i * 8 + 4);
        }

        for (i=0; i<nb; i++)
            dxStepBody(body[i],stepsize);

        // TODO: [INVESTIAGE] Unknown for cycle
        // Need resolve what cycle here in source
        // see dInternalStepIsland_x2:1393

        for (i=0; i<nb; i++) {
            body[i]->facc[0] = 0;
            body[i]->facc[1] = 0;
            body[i]->facc[2] = 0;
            body[i]->facc[3] = 0;
            body[i]->tacc[0] = 0;
            body[i]->tacc[1] = 0;
            body[i]->tacc[2] = 0;
            body[i]->tacc[3] = 0;
        }
    };

    void __fastcall dSolveL1 (const float *L, float *B, int n, int lskip1) {
        /* declare variables - Z matrix, p and q vectors, etc */
        float Z11,Z21,Z31,Z41,p1,q1,p2,p3,p4,*ex;
        const float *ell;
        int lskip2,lskip3,i,j;
        /* compute lskip values */
        lskip2 = 2*lskip1;
        lskip3 = 3*lskip1;
        /* compute all 4 x 1 blocks of X */
        for (i=0; i <= n-4; i+=4) {
            /* compute all 4 x 1 block of X, from rows i..i+4-1 */
            /* set the Z matrix to 0 */
            Z11=0;
            Z21=0;
            Z31=0;
            Z41=0;
            ell = L + i*lskip1;
            ex = B;
            /* the inner loop that computes outer products and adds them to Z */
            for (j=i-12; j >= 0; j -= 12) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                p2=ell[lskip1];
                p3=ell[lskip2];
                p4=ell[lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[1];
                q1=ex[1];
                p2=ell[1+lskip1];
                p3=ell[1+lskip2];
                p4=ell[1+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[2];
                q1=ex[2];
                p2=ell[2+lskip1];
                p3=ell[2+lskip2];
                p4=ell[2+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[3];
                q1=ex[3];
                p2=ell[3+lskip1];
                p3=ell[3+lskip2];
                p4=ell[3+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[4];
                q1=ex[4];
                p2=ell[4+lskip1];
                p3=ell[4+lskip2];
                p4=ell[4+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[5];
                q1=ex[5];
                p2=ell[5+lskip1];
                p3=ell[5+lskip2];
                p4=ell[5+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[6];
                q1=ex[6];
                p2=ell[6+lskip1];
                p3=ell[6+lskip2];
                p4=ell[6+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[7];
                q1=ex[7];
                p2=ell[7+lskip1];
                p3=ell[7+lskip2];
                p4=ell[7+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[8];
                q1=ex[8];
                p2=ell[8+lskip1];
                p3=ell[8+lskip2];
                p4=ell[8+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[9];
                q1=ex[9];
                p2=ell[9+lskip1];
                p3=ell[9+lskip2];
                p4=ell[9+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[10];
                q1=ex[10];
                p2=ell[10+lskip1];
                p3=ell[10+lskip2];
                p4=ell[10+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* load p and q values */
                p1=ell[11];
                q1=ex[11];
                p2=ell[11+lskip1];
                p3=ell[11+lskip2];
                p4=ell[11+lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* advance pointers */
                ell += 12;
                ex += 12;
                /* end of inner loop */
            }
            /* compute left-over iterations */
            j += 12;
            for (; j > 0; j--) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                p2=ell[lskip1];
                p3=ell[lskip2];
                p4=ell[lskip3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;
                /* advance pointers */
                ell += 1;
                ex += 1;
            }
            /* finish computing the X(i) block */
            Z11 = ex[0] - Z11;
            ex[0] = Z11;
            p1 = ell[lskip1];
            Z21 = ex[1] - Z21 - p1*Z11;
            ex[1] = Z21;
            p1 = ell[lskip2];
            p2 = ell[1+lskip2];
            Z31 = ex[2] - Z31 - p1*Z11 - p2*Z21;
            ex[2] = Z31;
            p1 = ell[lskip3];
            p2 = ell[1+lskip3];
            p3 = ell[2+lskip3];
            Z41 = ex[3] - Z41 - p1*Z11 - p2*Z21 - p3*Z31;
            ex[3] = Z41;
            /* end of outer loop */
        }
        /* compute rows at end that are not a multiple of block size */
        for (; i < n; i++) {
            /* compute all 1 x 1 block of X, from rows i..i+1-1 */
            /* set the Z matrix to 0 */
            Z11=0;
            ell = L + i*lskip1;
            ex = B;
            /* the inner loop that computes outer products and adds them to Z */
            for (j=i-12; j >= 0; j -= 12) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[1];
                q1=ex[1];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[2];
                q1=ex[2];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[3];
                q1=ex[3];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[4];
                q1=ex[4];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[5];
                q1=ex[5];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[6];
                q1=ex[6];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[7];
                q1=ex[7];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[8];
                q1=ex[8];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[9];
                q1=ex[9];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[10];
                q1=ex[10];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* load p and q values */
                p1=ell[11];
                q1=ex[11];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* advance pointers */
                ell += 12;
                ex += 12;
                /* end of inner loop */
            }
            /* compute left-over iterations */
            j += 12;
            for (; j > 0; j--) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                /* advance pointers */
                ell += 1;
                ex += 1;
            }
            /* finish computing the X(i) block */
            Z11 = ex[0] - Z11;
            ex[0] = Z11;
        }
    }

    void __fastcall dSolveL1T (const float *L, float *B, int n, int lskip1) {
        /* declare variables - Z matrix, p and q vectors, etc */
        float Z11,m11,Z21,m21,Z31,m31,Z41,m41,p1,q1,p2,p3,p4,*ex;
        const float *ell;
        int lskip2,lskip3,i,j;
        /* special handling for L and B because we're solving L1 *transpose* */
        L = L + (n-1)*(lskip1+1);
        B = B + n-1;
        lskip1 = -lskip1;
        /* compute lskip values */
        lskip2 = 2*lskip1;
        lskip3 = 3*lskip1;
        /* compute all 4 x 1 blocks of X */
        for (i=0; i <= n-4; i+=4) {
            /* compute all 4 x 1 block of X, from rows i..i+4-1 */
            /* set the Z matrix to 0 */
            Z11=0;
            Z21=0;
            Z31=0;
            Z41=0;
            ell = L - i;
            ex = B;
            /* the inner loop that computes outer products and adds them to Z */
            for (j=i-4; j >= 0; j -= 4) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                p2=ell[-1];
                p3=ell[-2];
                p4=ell[-3];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                m21 = p2 * q1;
                m31 = p3 * q1;
                m41 = p4 * q1;
                ell += lskip1;
                Z11 += m11;
                Z21 += m21;
                Z31 += m31;
                Z41 += m41;
                /* load p and q values */
                p1=ell[0];
                q1=ex[-1];
                p2=ell[-1];
                p3=ell[-2];
                p4=ell[-3];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                m21 = p2 * q1;
                m31 = p3 * q1;
                m41 = p4 * q1;
                ell += lskip1;
                Z11 += m11;
                Z21 += m21;
                Z31 += m31;
                Z41 += m41;
                /* load p and q values */
                p1=ell[0];
                q1=ex[-2];
                p2=ell[-1];
                p3=ell[-2];
                p4=ell[-3];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                m21 = p2 * q1;
                m31 = p3 * q1;
                m41 = p4 * q1;
                ell += lskip1;
                Z11 += m11;
                Z21 += m21;
                Z31 += m31;
                Z41 += m41;
                /* load p and q values */
                p1=ell[0];
                q1=ex[-3];
                p2=ell[-1];
                p3=ell[-2];
                p4=ell[-3];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                m21 = p2 * q1;
                m31 = p3 * q1;
                m41 = p4 * q1;
                ell += lskip1;
                ex -= 4;
                Z11 += m11;
                Z21 += m21;
                Z31 += m31;
                Z41 += m41;
                /* end of inner loop */
            }
            /* compute left-over iterations */
            j += 4;
            for (; j > 0; j--) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                p2=ell[-1];
                p3=ell[-2];
                p4=ell[-3];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                m21 = p2 * q1;
                m31 = p3 * q1;
                m41 = p4 * q1;
                ell += lskip1;
                ex -= 1;
                Z11 += m11;
                Z21 += m21;
                Z31 += m31;
                Z41 += m41;
            }
            /* finish computing the X(i) block */
            Z11 = ex[0] - Z11;
            ex[0] = Z11;
            p1 = ell[-1];
            Z21 = ex[-1] - Z21 - p1*Z11;
            ex[-1] = Z21;
            p1 = ell[-2];
            p2 = ell[-2+lskip1];
            Z31 = ex[-2] - Z31 - p1*Z11 - p2*Z21;
            ex[-2] = Z31;
            p1 = ell[-3];
            p2 = ell[-3+lskip1];
            p3 = ell[-3+lskip2];
            Z41 = ex[-3] - Z41 - p1*Z11 - p2*Z21 - p3*Z31;
            ex[-3] = Z41;
            /* end of outer loop */
        }
        /* compute rows at end that are not a multiple of block size */
        for (; i < n; i++) {
            /* compute all 1 x 1 block of X, from rows i..i+1-1 */
            /* set the Z matrix to 0 */
            Z11=0;
            ell = L - i;
            ex = B;
            /* the inner loop that computes outer products and adds them to Z */
            for (j=i-4; j >= 0; j -= 4) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                ell += lskip1;
                Z11 += m11;
                /* load p and q values */
                p1=ell[0];
                q1=ex[-1];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                ell += lskip1;
                Z11 += m11;
                /* load p and q values */
                p1=ell[0];
                q1=ex[-2];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                ell += lskip1;
                Z11 += m11;
                /* load p and q values */
                p1=ell[0];
                q1=ex[-3];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                ell += lskip1;
                ex -= 4;
                Z11 += m11;
                /* end of inner loop */
            }
            /* compute left-over iterations */
            j += 4;
            for (; j > 0; j--) {
                /* load p and q values */
                p1=ell[0];
                q1=ex[0];
                /* compute outer product and add it to the Z matrix */
                m11 = p1 * q1;
                ell += lskip1;
                ex -= 1;
                Z11 += m11;
            }
            /* finish computing the X(i) block */
            Z11 = ex[0] - Z11;
            ex[0] = Z11;
        }
    }

    float __fastcall dDot (const float *a, const float *b, int n) {
        size_t tail = n % 4;
        size_t base = n - tail;

        float sum = 0;
        for (size_t i = 0; i < base; i+=4) {
            __m128 x = _mm_load_ps(a + i);
            __m128 y = _mm_load_ps(b + i);
            __m128 s = _mm_dp_ps(x, y, 0x3F);
            sum += _mm_cvtss_f32(s);
        };

        for (; base < n; base++) {
            sum += a[base] * b[base];
        };

        return sum;
    }

    int __fastcall dFactorCholesky (float *A, int n) {
        int i,j,k,nskip;
        float sum,*a,*b,*aa,*bb,*cc;
        nskip = dPAD (n);

        HeapArray<float> recip (n);
        aa = A;
        for (i=0; i<n; i++) {
            bb = A;
            cc = A + i*nskip;

            for (j=0; j<i; j++) {
                sum = *cc;
                a = aa;
                b = bb;
                for (k=j; k; k--)
                    sum -= (*(a++))*(*(b++));
                *cc = sum * recip.ptr[j];
                bb += nskip;
                cc++;
            }

            sum = *cc;
            a = aa;

            for (k=i; k; k--, a++)
                sum -= (*a)*(*a);

            if (sum <= 0.0f)
                return 0;

            *cc = sqrtf(sum);
            recip.ptr[i] = 1.0f / *cc;
            aa += nskip;
        }
        return 1;
    };


    int __fastcall dIsPositiveDefinite (const float *A, int n) {
        int nskip = dPAD (n);
        HeapArray<float> Acopy (nskip * n);
        memcpy(Acopy.ptr, A, nskip * n * sizeof(float));
        return dFactorCholesky (Acopy.ptr, n);
    };

    void __fastcall dSolveCholesky (const float *L, float *b, int n) {
        int i,k,nskip;
        float sum;
        nskip = dPAD (n);
        HeapArray<float> y (n);

        for (i=0; i<n; i++) {
            sum = 0;
            for (k=0; k < i; k++) sum += L[i*nskip+k]*y.ptr[k];
            y.ptr[i] = (b[i]-sum)/L[i*nskip+i];
        }
        for (i=n-1; i >= 0; i--) {
            sum = 0;
            for (k=i+1; k < n; k++) sum += L[k*nskip+i]*b[k];
            b[i] = (y.ptr[i]-sum)/L[i*nskip+i];
        }
    }

    int __fastcall dInvertPDMatrix (const float *A, float *Ainv, int n)
    {
        int i,j,nskip;

        nskip = dPAD (n);

        HeapArray<float> L (nskip * n);
        memcpy(L.ptr,A,nskip*n*sizeof(float));
        HeapArray<float> x (n);

        if (dFactorCholesky(L.ptr, n) == 0) return 0;
            memset(Ainv, 0, n * nskip * sizeof(float));

            for (i=0; i<n; i++) {
                for (j=0; j<n; j++) x.ptr[j] = 0;
                    x.ptr[i] = 1;
                
                dSolveCholesky (L.ptr,x.ptr,n);

                for (j=0; j<n; j++)
                    Ainv[j*nskip+i] = x.ptr[j];
        }
        return 1;
    }

    #define M_SQRT1_2 0.7071067811865475244008443621048490f

    void __fastcall dLDLTAddTL (float *L, float *d, const float *a, int n, int nskip) {
        int j,p;
        float W11,W21,alpha1,alpha2,alphanew,gamma1,gamma2,k1,k2,Wp,ell,dee;

        if (n < 2) return;

        HeapArray<float> W1 (n);
        HeapArray<float> W2 (n);

        W1.ptr[0] = 0;
        W2.ptr[0] = 0;
        for (j=1; j<n; j++)
            W1.ptr[j] = W2.ptr[j] = a[j] * M_SQRT1_2;

        W11 = (0.5f * a[0]+1) * M_SQRT1_2;
        W21 = (0.5f * a[0]-1) * M_SQRT1_2;

        alpha1=1;
        alpha2=1;

        dee = d[0];
        alphanew = alpha1 + (W11*W11)*dee;
        dee /= alphanew;
        gamma1 = W11 * dee;
        dee *= alpha1;
        alpha1 = alphanew;
        alphanew = alpha2 - (W21*W21)*dee;
        dee /= alphanew;
        gamma2 = W21 * dee;
        alpha2 = alphanew;
        k1 = 1.0f - W21*gamma1;
        k2 = W21*gamma1*W11 - W21;
        for (p=1; p<n; p++) {
            Wp = W1.ptr[p];
            ell = L[p*nskip];
            W1.ptr[p] =    Wp - W11*ell;
            W2.ptr[p] = k1*Wp +  k2*ell;
        }

        for (j=1; j<n; j++) {
            dee = d[j];
            alphanew = alpha1 + (W1.ptr[j] * W1.ptr[j]) * dee;
            dee /= alphanew;
            gamma1 = W1.ptr[j] * dee;
            dee *= alpha1;
            alpha1 = alphanew;
            alphanew = alpha2 - (W2.ptr[j] * W2.ptr[j])*dee;
            dee /= alphanew;
            gamma2 = W2.ptr[j] * dee;
            dee *= alpha2;
            d[j] = dee;
            alpha2 = alphanew;

            k1 = W1.ptr[j];
            k2 = W2.ptr[j];
            for (p=j+1; p<n; p++) {
                ell = L[p*nskip+j];
                Wp = W1.ptr[p] - k1 * ell;
                ell += gamma1 * Wp;
                W1.ptr[p] = Wp;
                Wp = W2.ptr[p] - k2 * ell;
                ell -= gamma2 * Wp;
                W2.ptr[p] = Wp;
                L[p*nskip+j] = ell;
            }
        }
    }

    #define _GETA(i,j) (A[i][j])
    #define GETA(i,j) ((i > j) ? _GETA(i,j) : _GETA(j,i))

    void dRemoveRowCol (float *A, int n, int nskip, int r) {
        int i;
        if (r >= n-1) return;
        if (r > 0) {
            for (i=0; i<r; i++)
            memmove (A+i*nskip+r,A+i*nskip+r+1,(n-r-1)*sizeof(float));
            for (i=r; i<(n-1); i++)
            memcpy (A+i*nskip,A+i*nskip+nskip,r*sizeof(float));
        }
        for (i=r; i<(n-1); i++)
            memcpy (A+i*nskip+r,A+i*nskip+nskip+r+1,(n-r-1)*sizeof(float));
    }

    void __fastcall dLDLTRemove (float **A, const int *p, float *L, float *d, int n1, int n2, int r, int nskip) {
        int i;

        if (r==n2-1) {
            return;		// deleting last row/col is easy
        }
        else if (r==0) {
            HeapArray<float> a (n2);
            for (i=0; i<n2; i++) a.ptr[i] = -GETA(p[i],p[0]);
            a.ptr[0] += 1.0f;
            dLDLTAddTL (L,d,a.ptr,n2,nskip);
        }
        else {
            HeapArray<float> t (r);
            HeapArray<float> a (n2 - r);

            for (i=0; i<r; i++)
                t.ptr[i] = L[r*nskip+i] / d[i];

            for (i=0; i<(n2-r); i++)
                a.ptr[i] = dDot(L+(r+i)*nskip,t.ptr,r) - GETA(p[r+i],p[r]);
            
            a.ptr[0] += 1.0f;
            dLDLTAddTL (L + r*nskip+r, d+r, a.ptr, n2-r, nskip);
        }

        // snip out row/column r from L and d
        dRemoveRowCol(L,n2,nskip,r);
        if (r < (n2-1)) memmove (d+r,d+r+1,(n2-r-1)*sizeof(float));
    }

    typedef void* (__thiscall* PFNdObStack_Rewind)(dObStack* self);
    PFNdObStack_Rewind dObStack_Rewind = (PFNdObStack_Rewind) 0x008FC6F0;

    typedef void* (__thiscall* PFNdObStack_Next)(dObStack* self, int num_bytes);
    PFNdObStack_Next dObStack_Next = (PFNdObStack_Next) 0x008FC720;

    typedef void* (__thiscall* PFNdObStack_FreeAll)(dObStack* self);
    PFNdObStack_FreeAll dObStack_FreeAll = (PFNdObStack_FreeAll) 0x008FC6C0;

    void removeJointReferencesFromAttachedBodies(dxJoint *j) {
        for (int i=0; i<2; i++) {
            dxBody *body = j->node[i].body;
            if (body) {
                dxJointNode *n = body->firstjoint;
                dxJointNode *last = 0;
                while (n) {
                    if (n->joint == j) {
                        if (last) last->next = n->next;
                        else body->firstjoint = n->next;
                        break;
                    }
                    last = n;
                    n = n->next;
                }
            }
        }
        j->node[0].body = 0;
        j->node[0].next = 0;
        j->node[1].body = 0;
        j->node[1].next = 0;
    }

    void removeObjectFromList (dObject *obj) {
        if (obj->next) obj->next->tome = obj->tome;
        *(obj->tome) = obj->next;
        // safeguard
        obj->next = 0;
        obj->tome = 0;
    }


    void __fastcall dJointGroupEmpty (dJointGroupID group) {
        // the joints in this group are detached starting from the most recently
        // added (at the top of the stack). this helps ensure that the various
        // linked lists are not traversed too much, as the joints will hopefully
        // be at the start of those lists.
        // if any group joints have their world pointer set to 0, their world was
        // previously destroyed. no special handling is required for these joints.

        int i;
        HeapArray<dxJoint*> jlist (group->num);
        dxJoint *j = (dxJoint*) dObStack_Rewind(&group->stack);
        for (i=0; i < group->num; i++) {
            jlist.ptr[i] = j;
            j = (dxJoint*) dObStack_Next(&group->stack, j->vtable->size);
        }
        for (i=group->num-1; i >= 0; i--) {
            if (jlist.ptr[i]->world) {
            removeJointReferencesFromAttachedBodies(jlist.ptr[i]);
            removeObjectFromList (jlist.ptr[i]);
            jlist.ptr[i]->world->nj--;
            }
        }
        group->num = 0;
        dObStack_FreeAll(&group->stack);
    };

    void Apply() {
        routines::Redirect(0x2350, (void*) 0x008FF580, (void*) &dInternalStepIsland_x2);
        routines::Redirect(0x07B0, (void*) 0x00921A10, (void*) &dSolveL1);
        routines::Redirect(0x05B0, (void*) 0x00921460, (void*) &dSolveL1T);
        routines::Redirect(0x00D0, (void*) 0x009169E0, (void*) &dDot);
        routines::Redirect(0x0150, (void*) 0x0088B030, (void*) &dFactorCholesky);
        routines::Redirect(0x0330, (void*) 0x0088A400, (void*) &dSolveCholesky);
        routines::Redirect(0x0200, (void*) 0x0088B180, (void*) &dInvertPDMatrix);
        routines::Redirect(0x0060, (void*) 0x0088B380, (void*) &dIsPositiveDefinite);
        routines::Redirect(0x06E0, (void*) 0x0088A820, (void*) &dLDLTAddTL);
        routines::Redirect(0x03F0, (void*) 0x0088B3E0, (void*) &dLDLTRemove);
        routines::Redirect(0x00C0, (void*) 0x007C4FD0, (void*) &dJointGroupEmpty);
    };
};