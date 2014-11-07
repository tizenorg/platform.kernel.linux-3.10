/*
 * Samsung EXYNOS FIMC-IS (Imaging Subsystem) driver
 *
 * Copyright (C) 2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef FIMC_IS_FW_CONFIG_H
#define FIMC_IS_FW_CONFIG_H

/**
 * @brief Supported FIMC IS FW versions
 */
#define FIMC_IS_FW_V120 120 /* And higher */
#define FIMC_IS_FW_V130 130 /* And higher */


/**
 * @brief FIMC-IS firmware specific sub-blocks
 * 	  configuration options
 */
#define PARAM_OTF_INPUT 0x01
#define PARAM_DMA_INPUT 0x02
#define PARAM_ISP_CAM   0x03

/**
 * struct fimc_is_fw_mem - FW shared memory layout
 * @fw_size - total size of FW memory
 * @region_offset  - sub-block configuration region offset
 * @shared_offset  - MCUCTL shared memory offset
 * @dbg_offset	   - FW debug region offset
 * @camshot_size   - camera shot configuration size
 * @meta_type      - Metadata layout type (basic vs extended)
 */
struct fimc_is_fw_mem {
        unsigned long fw_size;
        unsigned long region_offset;
        unsigned long shared_offset;
        unsigned long dbg_offset;
        unsigned long shot_size;
        unsigned int  meta_type;
};

/**
 * @brief FIMC-IS supported firmware-specific operations
 */
struct fimc_is_fw_ops {
        /* Get firmware specific memory layout information */
        int (*mem_config) (struct fimc_is_fw_mem*);
        /* Tweak FIMC-IS subblock parameters specific for given firmware */
        int (*tweak_param) (unsigned int, void *, void*);
        /* Setup initial shot configuration */
        int (*config_shot) (void*);
};

extern const struct fimc_is_fw_ops *fimc_is_fw_ops;

/**
 * @brief Perform requested operation on current firmware
 * 	  (if supported)
 */
#define FIMC_IS_FW_CALL_OP(op, args...)		\
        ((fimc_is_fw_ops && fimc_is_fw_ops->op) \
        ? fimc_is_fw_ops->op(args) : -EINVAL)

/**
 * @brief Init supported operations for requested firmware version
 */
int fimc_is_fw_init_ops(unsigned int);

#endif /*FIMC_IS_FW_CONFIG_H*/
