#include "float.h"

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

        for (i=0; i<nj; i++) if (info.ptr[i].nub == info.ptr[i].m) {
            ofs.ptr[i] = m;
            m += info.ptr[i].m;
        }
        int nub = m;

        for (i=0; i<nj; i++) if (info.ptr[i].nub > 0 && info.ptr[i].nub < info.ptr[i].m) {
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

    void Apply() {
        routines::Redirect(0x2350, (void*) 0x008FF580, (void*) &dInternalStepIsland_x2);
    };
};