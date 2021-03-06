#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = gnu.targets.arm.M4{1,0,4.9,3
#
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/app_pm4g.om4g.dep
package/cfg/app_pm4g.om4g.dep: ;
endif

package/cfg/app_pm4g.om4g: | .interfaces
package/cfg/app_pm4g.om4g: package/cfg/app_pm4g.c package/cfg/app_pm4g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm4g $< ...
	$(gnu.targets.arm.M4.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mthumb -msoft-float -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__   -Dxdc_cfg__xheader__='"configPkg/package/cfg/app_pm4g.h"'  -Dxdc_target_name__=M4 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_9_3  -O2  -IC:/ti/tirtos_simplelink_2_01_00_03/products/bios_6_40_03_39/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/app_pm4g.om4g: export LD_LIBRARY_PATH=

package/cfg/app_pm4g.sm4g: | .interfaces
package/cfg/app_pm4g.sm4g: package/cfg/app_pm4g.c package/cfg/app_pm4g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm4g -S $< ...
	$(gnu.targets.arm.M4.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections -mcpu=cortex-m4 -mthumb -msoft-float -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__   -Dxdc_cfg__xheader__='"configPkg/package/cfg/app_pm4g.h"'  -Dxdc_target_name__=M4 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_9_3  -O2   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/cfg/app_pm4g.sm4g: export LD_LIBRARY_PATH=

clean,m4g ::
	-$(RM) package/cfg/app_pm4g.om4g
	-$(RM) package/cfg/app_pm4g.sm4g

app.pm4g: package/cfg/app_pm4g.om4g package/cfg/app_pm4g.mak

clean::
	-$(RM) package/cfg/app_pm4g.mak
