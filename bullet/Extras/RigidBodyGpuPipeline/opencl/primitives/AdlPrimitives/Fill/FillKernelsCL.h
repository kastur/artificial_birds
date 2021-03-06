static const char* fillKernelsCL= \
"/*\n"
"		2011 Takahiro Harada\n"
"*/\n"
"\n"
"#pragma OPENCL EXTENSION cl_amd_printf : enable\n"
"#pragma OPENCL EXTENSION cl_khr_local_int32_base_atomics : enable\n"
"\n"
"typedef unsigned int u32;\n"
"#define GET_GROUP_IDX get_group_id(0)\n"
"#define GET_LOCAL_IDX get_local_id(0)\n"
"#define GET_GLOBAL_IDX get_global_id(0)\n"
"#define GET_GROUP_SIZE get_local_size(0)\n"
"#define GROUP_LDS_BARRIER barrier(CLK_LOCAL_MEM_FENCE)\n"
"#define GROUP_MEM_FENCE mem_fence(CLK_LOCAL_MEM_FENCE)\n"
"#define AtomInc(x) atom_inc(&(x))\n"
"#define AtomInc1(x, out) out = atom_inc(&(x))\n"
"\n"
"#define make_uint4 (uint4)\n"
"#define make_uint2 (uint2)\n"
"#define make_int2 (int2)\n"
"\n"
"typedef struct\n"
"{\n"
"	int4 m_data;\n"
"	int m_offset;\n"
"	int m_n;\n"
"	int m_padding[2];\n"
"} ConstBuffer;\n"
"\n"
"\n"
"__kernel\n"
"__attribute__((reqd_work_group_size(64,1,1)))\n"
"void FillIntKernel(__global int* dstInt, \n"
"					ConstBuffer cb)\n"
"{\n"
"	int gIdx = GET_GLOBAL_IDX;\n"
"\n"
"	if( gIdx < cb.m_n )\n"
"	{\n"
"		dstInt[ cb.m_offset+gIdx ] = cb.m_data.x;\n"
"	}\n"
"}\n"
"\n"
"__kernel\n"
"__attribute__((reqd_work_group_size(64,1,1)))\n"
"void FillInt2Kernel(__global int2* dstInt2, \n"
"					ConstBuffer cb)\n"
"{\n"
"	int gIdx = GET_GLOBAL_IDX;\n"
"\n"
"	if( gIdx < cb.m_n )\n"
"	{\n"
"		dstInt2[ cb.m_offset+gIdx ] = make_int2( cb.m_data.x, cb.m_data.y );\n"
"	}\n"
"}\n"
"\n"
"__kernel\n"
"__attribute__((reqd_work_group_size(64,1,1)))\n"
"void FillInt4Kernel(__global int4* dstInt4, \n"
"					ConstBuffer cb)\n"
"{\n"
"	int gIdx = GET_GLOBAL_IDX;\n"
"\n"
"	if( gIdx < cb.m_n )\n"
"	{\n"
"		dstInt4[ cb.m_offset+gIdx ] = cb.m_data;\n"
"	}\n"
"}\n"
"\n"
;
