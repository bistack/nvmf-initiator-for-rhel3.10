/*
 * NVMe I/O command implementation.
 * Copyright (c) 2015-2016 HGST, a Western Digital Company.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/blkdev.h>
#include <linux/module.h>
#include "nvmet.h"

#ifdef HAVE_BIO_ENDIO_1_PARAM
static void nvmet_bio_done(struct bio *bio)
#else
static void nvmet_bio_done(struct bio *bio, int error)
#endif
{
	struct nvmet_req *req = bio->bi_private;

	nvmet_req_complete(req,
#ifdef HAVE_BLK_STATUS_T
		bio->bi_status ? NVME_SC_INTERNAL | NVME_SC_DNR : 0);
#elif defined(HAVE_STRUCT_BIO_BI_ERROR)
		bio->bi_error ? NVME_SC_INTERNAL | NVME_SC_DNR : 0);
#else
		error ? NVME_SC_INTERNAL | NVME_SC_DNR : 0);
#endif

	if (bio != &req->inline_bio)
		bio_put(bio);
}

static inline u32 nvmet_rw_len(struct nvmet_req *req)
{
	return ((u32)le16_to_cpu(req->cmd->rw.length) + 1) <<
			req->ns->blksize_shift;
}

static void nvmet_execute_rw(struct nvmet_req *req)
{
	int sg_cnt = req->sg_cnt;
	struct bio *bio = &req->inline_bio;
	struct scatterlist *sg;
	sector_t sector;
#ifdef HAVE_SUBMIT_BIO_1_PARAM
	blk_qc_t cookie;
#endif
	int op, op_flags = 0, i;

	if (!req->sg_cnt) {
		nvmet_req_complete(req, 0);
		return;
	}

	if (req->cmd->rw.opcode == nvme_cmd_write) {
		op = REQ_OP_WRITE;
#ifdef HAVE_REQ_IDLE
		op_flags = REQ_SYNC | REQ_IDLE;
#else
		op_flags = WRITE_ODIRECT;
#endif
		if (req->cmd->rw.control & cpu_to_le16(NVME_RW_FUA))
			op_flags |= REQ_FUA;
	} else {
		op = REQ_OP_READ;
	}

	sector = le64_to_cpu(req->cmd->rw.slba);
	sector <<= (req->ns->blksize_shift - 9);

#ifdef HAVE_BIO_INIT_3_PARAMS
	bio_init(bio, req->inline_bvec, ARRAY_SIZE(req->inline_bvec));
#else
	bio_init(bio);
	bio->bi_io_vec = req->inline_bvec;
	bio->bi_max_vecs = ARRAY_SIZE(req->inline_bvec);
#endif
#ifdef HAVE_BIO_BI_DISK
	bio_set_dev(bio, req->ns->bdev);
#else
	bio->bi_bdev = req->ns->bdev;
#endif
#ifdef HAVE_STRUCT_BIO_BI_ITER
	bio->bi_iter.bi_sector = sector;
#else
	bio->bi_sector = sector;
#endif
	bio->bi_private = req;
	bio->bi_end_io = nvmet_bio_done;
	bio_set_op_attrs(bio, op, op_flags);

#ifdef HAVE_RH7_STRUCT_BIO_AUX
	bio_init_aux(bio, &req->bio_aux);
#endif

	for_each_sg(req->sg, sg, req->sg_cnt, i) {
		while (bio_add_page(bio, sg_page(sg), sg->length, sg->offset)
				!= sg->length) {
			struct bio *prev = bio;

			bio = bio_alloc(GFP_KERNEL, min(sg_cnt, BIO_MAX_PAGES));
#ifdef HAVE_BIO_BI_DISK
			bio_set_dev(bio, req->ns->bdev);
#else
			bio->bi_bdev = req->ns->bdev;
#endif
#ifdef HAVE_STRUCT_BIO_BI_ITER
			bio->bi_iter.bi_sector = sector;
#else
			bio->bi_sector = sector;
#endif
			bio_set_op_attrs(bio, op, op_flags);

			bio_chain(bio, prev);
#ifdef HAVE_SUBMIT_BIO_1_PARAM
			submit_bio(prev);
#else
			submit_bio(bio_data_dir(prev), prev);
#endif
		}

		sector += sg->length >> 9;
		sg_cnt--;
	}

#ifdef HAVE_SUBMIT_BIO_1_PARAM
	cookie = submit_bio(bio);

#ifdef HAVE_BLK_MQ_POLL
	blk_mq_poll(bdev_get_queue(req->ns->bdev), cookie);
#elif defined(HAVE_BLK_POLL)
	blk_poll(bdev_get_queue(req->ns->bdev), cookie);
#endif /* HAVE_BLK_MQ_POLL */

#else
	submit_bio(bio_data_dir(bio), bio);
#endif /* HAVE_SUBMIT_BIO_1_PARAM */
}

static void nvmet_execute_flush(struct nvmet_req *req)
{
	struct bio *bio = &req->inline_bio;

#ifdef HAVE_BIO_INIT_3_PARAMS
	bio_init(bio, req->inline_bvec, ARRAY_SIZE(req->inline_bvec));
#else
	bio_init(bio);
	bio->bi_io_vec = req->inline_bvec;
	bio->bi_max_vecs = ARRAY_SIZE(req->inline_bvec);
#endif
#ifdef HAVE_BIO_BI_DISK
	bio_set_dev(bio, req->ns->bdev);
#else
	bio->bi_bdev = req->ns->bdev;
#endif
	bio->bi_private = req;
	bio->bi_end_io = nvmet_bio_done;
#ifdef HAVE_STRUCT_BIO_BI_OPF
	bio->bi_opf = REQ_OP_WRITE | REQ_PREFLUSH;
#else
	bio_set_op_attrs(bio, REQ_OP_WRITE, WRITE_FLUSH);
#endif

#ifdef HAVE_SUBMIT_BIO_1_PARAM
	submit_bio(bio);
#else
	submit_bio(bio_data_dir(bio), bio);
#endif
}

static u16 nvmet_discard_range(struct nvmet_ns *ns,
		struct nvme_dsm_range *range, struct bio **bio)
{
	int ret;

#ifdef HAVE___BLKDEV_ISSUE_DISCARD
	ret = __blkdev_issue_discard(ns->bdev,
			le64_to_cpu(range->slba) << (ns->blksize_shift - 9),
			le32_to_cpu(range->nlb) << (ns->blksize_shift - 9),
			GFP_KERNEL, 0, bio);
#else
	ret = blkdev_issue_discard(ns->bdev,
			le64_to_cpu(range->slba) << (ns->blksize_shift - 9),
			le32_to_cpu(range->nlb) << (ns->blksize_shift - 9),
			GFP_KERNEL, 0);
#endif
	if (ret && ret != -EOPNOTSUPP)
		return NVME_SC_INTERNAL | NVME_SC_DNR;
	return 0;
}

static void nvmet_execute_discard(struct nvmet_req *req)
{
	struct nvme_dsm_range range;
	struct bio *bio = NULL;
	int i;
	u16 status;

	for (i = 0; i <= le32_to_cpu(req->cmd->dsm.nr); i++) {
		status = nvmet_copy_from_sgl(req, i * sizeof(range), &range,
				sizeof(range));
		if (status)
			break;

		status = nvmet_discard_range(req->ns, &range, &bio);
		if (status)
			break;
	}

	if (bio) {
		bio->bi_private = req;
		bio->bi_end_io = nvmet_bio_done;
		if (status) {
#ifdef HAVE_BLK_STATUS_T
			bio->bi_status = BLK_STS_IOERR;
#elif defined(HAVE_STRUCT_BIO_BI_ERROR)
			bio->bi_error = -EIO;
#endif
#ifdef HAVE_BIO_ENDIO_1_PARAM
			bio_endio(bio);
#else
			bio_endio(bio, -EIO);
#endif
		} else {
#ifdef HAVE_SUBMIT_BIO_1_PARAM
			submit_bio(bio);
#else
			submit_bio(bio_data_dir(bio), bio);
#endif
		}
	} else {
		nvmet_req_complete(req, status);
	}
}

static void nvmet_execute_dsm(struct nvmet_req *req)
{
	switch (le32_to_cpu(req->cmd->dsm.attributes)) {
	case NVME_DSMGMT_AD:
		nvmet_execute_discard(req);
		return;
	case NVME_DSMGMT_IDR:
	case NVME_DSMGMT_IDW:
	default:
		/* Not supported yet */
		nvmet_req_complete(req, 0);
		return;
	}
}

#ifdef HAVE_BLKDEV_ISSUE_ZEROOUT
static void nvmet_execute_write_zeroes(struct nvmet_req *req)
{
	struct nvme_write_zeroes_cmd *write_zeroes = &req->cmd->write_zeroes;
	struct bio *bio = NULL;
	u16 status = NVME_SC_SUCCESS;
	sector_t sector;
	sector_t nr_sector;

	sector = le64_to_cpu(write_zeroes->slba) <<
		(req->ns->blksize_shift - 9);
	nr_sector = (((sector_t)le16_to_cpu(write_zeroes->length) + 1) <<
		(req->ns->blksize_shift - 9));

#ifdef CONFIG_COMPAT_IS_BLKDEV_ISSUE_ZEROOUT_HAS_FLAGS
	if (__blkdev_issue_zeroout(req->ns->bdev, sector, nr_sector,
				GFP_KERNEL, &bio, 0))
#else
	if (__blkdev_issue_zeroout(req->ns->bdev, sector, nr_sector,
				GFP_KERNEL, &bio, true))

#endif
		status = NVME_SC_INTERNAL | NVME_SC_DNR;

	if (bio) {
		bio->bi_private = req;
		bio->bi_end_io = nvmet_bio_done;
#ifdef HAVE_SUBMIT_BIO_1_PARAM
		submit_bio(bio);
#else
		submit_bio(bio_data_dir(bio), bio);
#endif
	} else {
		nvmet_req_complete(req, status);
	}
}
#endif

u16 nvmet_parse_io_cmd(struct nvmet_req *req)
{
	struct nvme_command *cmd = req->cmd;
	u16 ret;

	ret = nvmet_check_ctrl_status(req, cmd);
	if (unlikely(ret))
		return ret;

	req->ns = nvmet_find_namespace(req->sq->ctrl, cmd->rw.nsid);
	if (unlikely(!req->ns))
		return NVME_SC_INVALID_NS | NVME_SC_DNR;

	switch (cmd->common.opcode) {
	case nvme_cmd_read:
	case nvme_cmd_write:
		req->execute = nvmet_execute_rw;
		req->data_len = nvmet_rw_len(req);
		return 0;
	case nvme_cmd_flush:
		req->execute = nvmet_execute_flush;
		req->data_len = 0;
		return 0;
	case nvme_cmd_dsm:
		req->execute = nvmet_execute_dsm;
		req->data_len = (le32_to_cpu(cmd->dsm.nr) + 1) *
			sizeof(struct nvme_dsm_range);
		return 0;
#ifdef HAVE_BLKDEV_ISSUE_ZEROOUT
	case nvme_cmd_write_zeroes:
		req->execute = nvmet_execute_write_zeroes;
		return 0;
#endif
	default:
		pr_err("unhandled cmd %d on qid %d\n", cmd->common.opcode,
		       req->sq->qid);
		return NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
	}
}
