*** Settings ***
Suite Setup                   Setup
Suite Teardown                Teardown
Test Setup                    Reset Emulation
Test Teardown                 Test Teardown
Resource                      ${RENODEKEYWORDS}

*** Variables ***
${SCRIPT}                     ${CURDIR}/leon3_rtems.resc
${UART}                       sysbus.uart
${PROMPT}                     SHLL [/] # 

*** Keywords ***
Prepare Sample
    [Arguments]  ${sample_name}

    Execute Command           $prom="${CURDIR}/grlib-gpl-2021.2-b4267/software/leon3/prom.bin"
    Execute Command           $bin="${CURDIR}/rcc-1.3.0-gcc/src/samples/bin/leon3/${sample_name}"
    Execute Script            ${SCRIPT}
    Set Default Uart Timeout  60
    Create Terminal Tester    ${UART}

*** Test Cases ***
Should Boot RTEMS
    [Documentation]           Boots RTEMS on the Leon3 platform.
    [Tags]                    rtems  uart
    Prepare Sample            rtems-shell

    Start Emulation

    Wait For Prompt On Uart   ${PROMPT}

    Provides                  booted-rtems

Should Print System Information
    [Documentation]           Tests shell responsiveness in RTEMS on the Leon3 platform.
    [Tags]                    rtems  uart
    Requires                  booted-rtems

    Write Line To Uart        drvmgr
    Wait For Line On Uart     NO DEVICES FAILED TO INITIALIZE

Should Pass Timer Test
    [Documentation]           Using the timer library TLIB provided by the LEON3 BSP.
    [Tags]                    rtems  uart
    Prepare Sample            rtems-tlib

    Start Emulation

    Wait For Line On Uart     Timer test OK

Should Print Hello World
    [Documentation]           Run basic hello world sample.
    [Tags]                    rtems  uart
    Prepare Sample            rtems-hello

    Start Emulation

    Wait For Line On Uart     Hello World over printk() on Debug console

Should Initialize Network
    [Documentation]           Run ttcp sample.
    [Tags]                    rtems  uart
    Prepare Sample            rtems-ttcp

    Start Emulation

    Wait For Line On Uart     Initializing network DONE