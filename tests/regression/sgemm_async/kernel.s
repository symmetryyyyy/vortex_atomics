	.text
	.attribute	4, 16
	.attribute	5, "rv32i2p1_m2p0_a2p1_f2p2_zicsr2p0"
	.file	"kernel.cpp"
	.globl	_Z11kernel_bodyP12kernel_arg_t  # -- Begin function _Z11kernel_bodyP12kernel_arg_t
	.p2align	2
	.type	_Z11kernel_bodyP12kernel_arg_t,@function
_Z11kernel_bodyP12kernel_arg_t:         # @_Z11kernel_bodyP12kernel_arg_t
	.cfi_startproc
# %bb.0:
	addi	sp, sp, -16
	.cfi_def_cfa_offset 16
	sw	s0, 12(sp)                      # 4-byte Folded Spill
	sw	s1, 8(sp)                       # 4-byte Folded Spill
	sw	s2, 4(sp)                       # 4-byte Folded Spill
	sw	s3, 0(sp)                       # 4-byte Folded Spill
	.cfi_offset s0, -4
	.cfi_offset s1, -8
	.cfi_offset s2, -12
	.cfi_offset s3, -16
	lw	a5, 24(a0)
	lw	a6, 32(a0)
	lw	a1, 40(a0)
	#APP
	csrr	t0, nw
	#NO_APP
	lui	a7, %hi(blockDim)
.Lpcrel_hi0:
	auipc	a2, %tls_ie_pcrel_hi(blockIdx)
	lw	t4, %pcrel_lo(.Lpcrel_hi0)(a2)
.Lpcrel_hi1:
	auipc	a2, %tls_ie_pcrel_hi(threadIdx)
	lw	t3, %pcrel_lo(.Lpcrel_hi1)(a2)
	addi	a2, a7, %lo(blockDim)
	lw	t2, 4(a2)
	add	t4, t4, tp
	add	t3, t3, tp
	lw	a2, 4(t4)
	lw	t1, 4(t3)
	lw	a4, 16(a0)
	mul	a2, a2, t2
	add	a2, a2, t1
	slli	a3, a2, 2
	beqz	a4, .LBB0_6
# %bb.1:
.Lpcrel_hi2:
	auipc	a2, %tls_ie_pcrel_hi(__local_group_id)
	lw	a2, %pcrel_lo(.Lpcrel_hi2)(a2)
	add	a2, a2, tp
	lw	a0, 20(a0)
	lw	t4, 0(t4)
	lw	t5, %lo(blockDim)(a7)
	lw	a7, 0(t3)
	add	a6, a6, a3
	lw	t3, 0(a2)
	mul	t4, t4, t5
	add	t6, t4, a7
	mul	t2, t2, t5
	mul	t3, t2, t3
	slli	t3, t3, 3
	add	t5, t0, t3
	slli	t2, t2, 2
	add	t4, t5, t2
	mul	t2, t6, a4
	slli	t0, t2, 2
	slli	t6, t1, 2
	add	a5, a5, t6
	add	a5, a5, t0
	mul	s0, a7, a0
	add	t1, t1, s0
	slli	t1, t1, 2
	add	t0, t5, t1
	add	t1, t4, t1
	beqz	a0, .LBB0_8
# %bb.2:
	li	t3, 0
	add	t4, t4, t6
	slli	s0, s0, 2
	add	t5, t5, s0
	slli	t6, a0, 2
	fmv.w.x	fa5, zero
	lui	s0, %hi(__warps_per_group)
.LBB0_3:                                # =>This Loop Header: Depth=1
                                        #     Child Loop BB0_4 Depth 2
	slli	s1, t3, 2
	add	s1, a5, s1
	flw	fa4, 0(s1)
	fsw	fa4, 0(t0)
	add	s1, t3, a7
	mul	s1, s1, a4
	slli	s1, s1, 2
	add	s1, a6, s1
	flw	fa4, 0(s1)
	lw	s1, 0(a2)
	lw	s2, %lo(__warps_per_group)(s0)
	fsw	fa4, 0(t1)
	#APP
	.insn r 11, 6, 0, zero, s1, s2
	#NO_APP
	mv	s1, t5
	mv	s2, t4
	mv	s3, a0
.LBB0_4:                                #   Parent Loop BB0_3 Depth=1
                                        # =>  This Inner Loop Header: Depth=2
	flw	fa4, 0(s1)
	flw	fa3, 0(s2)
	fmadd.s	fa5, fa4, fa3, fa5
	addi	s3, s3, -1
	add	s2, s2, t6
	addi	s1, s1, 4
	bnez	s3, .LBB0_4
# %bb.5:                                #   in Loop: Header=BB0_3 Depth=1
	lw	s1, 0(a2)
	lw	s2, %lo(__warps_per_group)(s0)
	#APP
	.insn r 11, 7, 0, zero, s1, s2
	#NO_APP
	add	t3, t3, a0
	bltu	t3, a4, .LBB0_3
	j	.LBB0_7
.LBB0_6:
	li	t2, 0
	fmv.w.x	fa5, zero
.LBB0_7:
	slli	t2, t2, 2
	add	a1, a1, t2
	add	a1, a1, a3
	fsw	fa5, 0(a1)
	lw	s0, 12(sp)                      # 4-byte Folded Reload
	lw	s1, 8(sp)                       # 4-byte Folded Reload
	lw	s2, 4(sp)                       # 4-byte Folded Reload
	lw	s3, 0(sp)                       # 4-byte Folded Reload
	addi	sp, sp, 16
	ret
.LBB0_8:
	mul	a0, a7, a4
	slli	a0, a0, 2
	add	a6, a6, a0
	lui	a0, %hi(__warps_per_group)
.LBB0_9:                                # =>This Inner Loop Header: Depth=1
	flw	fa5, 0(a5)
	fsw	fa5, 0(t0)
	flw	fa5, 0(a6)
	lw	a1, 0(a2)
	lw	a3, %lo(__warps_per_group)(a0)
	fsw	fa5, 0(t1)
	#APP
	.insn r 11, 6, 0, zero, a1, a3
	#NO_APP
	lw	a1, 0(a2)
	lw	a3, %lo(__warps_per_group)(a0)
	#APP
	.insn r 11, 7, 0, zero, a1, a3
	#NO_APP
	j	.LBB0_9
.Lfunc_end0:
	.size	_Z11kernel_bodyP12kernel_arg_t, .Lfunc_end0-_Z11kernel_bodyP12kernel_arg_t
	.cfi_endproc
                                        # -- End function
	.globl	main                            # -- Begin function main
	.p2align	2
	.type	main,@function
main:                                   # @main
	.cfi_startproc
# %bb.0:
	#APP
	csrr	a1, mscratch
	#NO_APP
	addi	a2, a1, 8
	lui	a0, %hi(_Z11kernel_bodyP12kernel_arg_t)
	addi	a3, a0, %lo(_Z11kernel_bodyP12kernel_arg_t)
	li	a0, 2
	mv	a4, a1
	tail	vx_spawn_threads
.Lfunc_end1:
	.size	main, .Lfunc_end1-main
	.cfi_endproc
                                        # -- End function
	.ident	"clang version 18.1.7 (https://github.com/vortexgpgpu/llvm.git b115a172abc24683b2730b5b601f34e27fe19d93)"
	.section	".note.GNU-stack","",@progbits
	.addrsig
	.addrsig_sym _Z11kernel_bodyP12kernel_arg_t
