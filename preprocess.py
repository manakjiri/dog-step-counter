d = """
this is a preprocessor that enables a C++ development workflow for STM32CubeIDE
"""

import argparse, re, subprocess
from pathlib import Path
from textwrap import TextWrapper
import traceback

parser = argparse.ArgumentParser(description=d)
parser.add_argument('ioc_file', help='path to the .ioc CubeMX file of your project')
parser.add_argument('--no-main-template', help='disables the automatic main.cpp template creation', action='store_true')
parser.add_argument('--no-patch', help='disables the application of patches', action='store_true')
args, unknown = parser.parse_known_args()

IOC_PATH = Path(args.ioc_file)
assert IOC_PATH.exists(), f'ioc file "{IOC_PATH}" not found'
BASE_PATH = IOC_PATH.parent

MAIN_C_PATH = BASE_PATH.joinpath('Core/Src/main.c')
MAIN_H_PATH = BASE_PATH.joinpath('Core/Inc/main.h')
MAIN_CPP_PATH = BASE_PATH.joinpath('Core/Src/main.cpp')

INCLUDE_REPLACE = list(BASE_PATH.glob('Core/Src/stm32*.c'))
INCLUDE_REPLACE.append(BASE_PATH.joinpath('USBPD/usbpd_dpm_user.c'))
INCLUDE_REPLACE.extend(BASE_PATH.glob('FATFS/**/*'))

PATCHES_PATHS = list(BASE_PATH.glob('patches/*.patch'))
PATCH_EXEC_PATHS = [Path('/usr/bin/patch'), Path('/run/host/usr/bin/patch')]

COMMENT_OPEN = '/*'
COMMENT_CLOSE = '*/'
SCRIPT_NAME = Path(__file__).name
SCRIPT_SETUP_DOCS = 'https://z-bionics.atlassian.net/l/c/sw7HmYU5'

INIT_H_TEMPLATE = \
["""
#ifndef __INIT_H
#define __INIT_H

#ifdef __cplusplus
extern "C" {
#endif
""","""
#ifdef __cplusplus
}
#endif

#endif
"""]

MAIN_CPP_TEMPLATE = \
"""
#include "init.h"


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize peripherals */
	MX_GPIO_Init();

    /* CALL REMAINING INIT FUNCTIONS HERE, they are defined in init.c and should 
     * be declared in init.h, which is included at the beginning of this file.
     * this is a template file and does not get modified in any way, you are
     * responsible for calling those functions. */

    while (1)
    {


    }
}
"""

def info(*args, **kwargs):
    pre = f'{SCRIPT_NAME}: '
    wrapper = TextWrapper(
        initial_indent=pre,
        subsequent_indent=len(pre) * ' ',
        fix_sentence_endings=True,
        break_long_words=False,
        width=120
    )
    msg = wrapper.wrap(" ".join(map(str, args)))
    print('\n'.join(msg), **kwargs)

def warning(msg: str):
    info(f'warning: {msg}')

def warning_not_found(p: Path):
    warning(f'{p} not found')

def get_disclaimer():
    text = f'##### THIS FILE GETS MODIFIED OR OVERWRITTEN BY THE {SCRIPT_NAME} SCRIPT BEFORE BUILD #####'
    return f'{COMMENT_OPEN} {((len(text) - len(COMMENT_OPEN) - 1) * "#")}\n{text}\n'\
           f'{((len(text) - len(COMMENT_CLOSE) - 1) * "#")} {COMMENT_CLOSE}'

def run_command(cmd: str):
    info(f'running subprocess "{cmd}"')
    proc = subprocess.run(cmd.split(), capture_output=True, text=True)
    if proc.stdout:
        info('subprocess stdout:', proc.stdout.rstrip())
    if proc.stderr:
        info('subprocess stderr:', proc.stderr.rstrip())
    return proc

def main():
    # read the main.c file, change it so that it includes "init.h"
    # instead of "main.h" and save it as init.c, finally delete the original main.c
    if MAIN_C_PATH.exists() and MAIN_H_PATH.exists():
        
        # read the main.c file, this contains all the init functions
        info(f'loading the "{MAIN_C_PATH}" file')
        main_c = MAIN_C_PATH.read_text()
        main_c = main_c.replace('#include "main.h', '#include "init.h')
        main_c = main_c.replace('static void', 'void')

        info(f'loading the "{MAIN_H_PATH}" file')
        main_h = MAIN_H_PATH.read_text()

        info(f'extracting function definitions, pin definitions and handle declarations')
        
        # we want to extract all the init function definitions
        init_fn_defs = re.findall(r'void \w*\(void\)', main_c)

        # and the HAL peripheral handles
        periph_handles = re.findall(r'\w*_HandleTypeDef h\w*;', main_c)
        # and pin/port name definitions
        pin_defs = re.findall(r'#define \w* \w*', main_h)
        # and finally the HAL lib include
        hal_lib = re.findall(r'#include "stm32\w*_hal.h"', main_h)

        # create the contents of the init.h file and save it
        init_h = get_disclaimer()
        init_h += INIT_H_TEMPLATE[0]
        init_h += '\n\n'
        init_h += '\n'.join(hal_lib)
        init_h += '\n\n'
        init_h += '\n'.join(pin_defs)
        init_h += '\n\n'

        for ph in periph_handles:
            init_h += f'extern {ph}\n'
            # cut out the periphery name from the handle, ie "TIM_HandleTypeDef htim1;" becomes "TIM1"
            name = str(ph[ph.rfind('h')+1 : ph.rfind(';')])
            init_h += f'#define ZST_HAL_{name.upper()}_PRESENT\n'

        init_h += '\n'

        for fn in init_fn_defs:
            if not fn in init_h:
                init_h += f'{fn};\n'
        
        # this function is defined in stm32g4xx_hal_msp.c, so we miss it
        init_h += '\n#ifdef HAL_TIM_MODULE_ENABLED\n'
        init_h += 'void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);\n'
        init_h += '#endif\n'
        
        init_h += '\n'
        init_h += INIT_H_TEMPLATE[1]
        info('saving the "init.h" file')
        MAIN_H_PATH.parent.joinpath('init.h').write_text(init_h)
        
        # save back the modified main.c file as init.c
        init_c = get_disclaimer() + '\n\n' + main_c
        info('saving the "init.c" file')
        MAIN_C_PATH.parent.joinpath('init.c').write_text(init_c)
        
        # delete the original main.c and main.h files
        info(f'deleting original {MAIN_C_PATH} and {MAIN_H_PATH}')
        MAIN_C_PATH.unlink()
        MAIN_H_PATH.unlink()
    
    else:
        info(f'{MAIN_C_PATH} or {MAIN_H_PATH} not found, nothing to extract')


    for p in INCLUDE_REPLACE:
        if p.exists() and p.is_file():
            file = p.read_text()
            if '#include "main.h' in file:
                info(f'Replacing the include directive in {p}')
                file = file.replace('#include "main.h', '#include "init.h')
                if not get_disclaimer() in file:
                    file = get_disclaimer() + '\n\n' + file
                p.write_text(file)
        else:
            warning_not_found(p)


    if not MAIN_CPP_PATH.exists() and not args.no_main_template:
        info(
            f'"{MAIN_CPP_PATH}" not found, creating a template, you can disable this',
            f'using the "--no-main-template" flag, see "python3 {SCRIPT_NAME} --help"'
        )
        MAIN_CPP_PATH.write_text(MAIN_CPP_TEMPLATE)

    if not args.no_patch:
        
        patch_path = ''
        for p in PATCH_EXEC_PATHS:
            if p.is_file():
                patch_path = p
                break
        else:
            warning(f'unable to apply patches: the patch program is unavailable, tried checking: {PATCH_EXEC_PATHS}')

        if patch_path:
            for p in PATCHES_PATHS:
                info(f'attempting to apply the "{p}" patch, you can disable this using the "--no-patch" flag')
                opts = '--unified --no-backup-if-mismatch -p0 --forward'
                paths = f'-i {p.absolute()} -d {BASE_PATH.absolute()}'
                
                dry = run_command(f'{patch_path} {opts} --dry-run {paths}')
                
                if dry.returncode == 0: #pset.can_patch():
                    run_command(f'{patch_path} {opts} {paths}')

                    for rej in BASE_PATH.glob('**/*.rej'):
                        rej_target = p.parent.joinpath(rej.name)
                        warning(f'moving {rej} to {rej_target}')
                        rej.rename(rej_target)
        

if __name__ == '__main__':
    try:
        main()
    except Exception as exception:
        info(
            f'Unhandled Exception occurred. There may have been an update of the {SCRIPT_NAME} script,',
            f'please check the setup documentation at {SCRIPT_SETUP_DOCS} and the README.md file.'
        )
        info('Exception traceback:')
        print(traceback.format_exc())

