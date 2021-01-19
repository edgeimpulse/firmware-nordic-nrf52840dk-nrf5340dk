
/* Include ----------------------------------------------------------------- */
#include "repl.h"
#include <stdio.h>
#include <string>
#include <vector>
#include "ei_classifier_porting.h"
#include <sys/printk.h>
#include "ei_device_nordic_nrf52.h"

#define ei_putc(c) uart_putchar(c)


extern bool ei_at_cmd_handle(const char *cmd_in);

using namespace std;

class ReplBuffer {
public:
    ReplBuffer() {
        position = 0;
    }

    void clear() {
        buffer.clear();
        position = 0;
    }

    void add(string s) {
        for(string::iterator it = s.begin(); it != s.end(); ++it) {
            buffer.insert(buffer.begin() + position, *it);
            position++;
        }
    }

    void add(char c) {
        buffer.insert(buffer.begin() + position, c);
        position++;
    }

    vector<char>::iterator begin() {
        return buffer.begin();
    }

    vector<char>::iterator end() {
        return buffer.end();
    }

    size_t getPosition() {
        return position;
    }

    void setPosition(size_t pos) {
        position = pos;
    }

    size_t size() {
        return buffer.size();
    }

private:
    vector<char> buffer;
    size_t position;
};

/* Private variables ------------------------------------------------------- */
static bool inControlChar = false;
static vector<char> controlSequence;
static vector<string> history;
static size_t historyPosition;
static ReplBuffer buffer;

/* Private functions prototypes -------------------------------------------- */
static void handleBackspace(void);
static void runBuffer(void);

void rx_callback(char c) {
    // control characters start with 0x1b and end with a-zA-Z
    if (inControlChar) {

        controlSequence.push_back(c);

        // if a-zA-Z then it's the last one in the control char...
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
            inControlChar = false;

            // up
            if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x41) {
                ei_printf("\033[u"); // restore current position

                if (historyPosition == 0) {
                    // cannot do...
                }
                else {
                    historyPosition--;
                    // reset cursor to 0, do \r, then write the new command...
                    ei_printf("\33[2K\r> %s", history[historyPosition].c_str());

                    buffer.clear();
                    buffer.add(history[historyPosition]);
                }
            }
            // down
            else if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x42) {
                ei_printf("\033[u"); // restore current position

                if (historyPosition == history.size()) {
                    // no-op
                }
                else if (historyPosition == history.size() - 1) {
                    historyPosition++;

                    // put empty
                    // reset cursor to 0, do \r, then write the new command...
                    ei_printf("\33[2K\r> ");

                    buffer.clear();
                }
                else {
                    historyPosition++;
                    // reset cursor to 0, do \r, then write the new command...
                    ei_printf("\33[2K\r> %s", history[historyPosition].c_str());

                    buffer.clear();
                    buffer.add(history[historyPosition]);
                }
            }
            // left
            else if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x44) {
                size_t curr = buffer.getPosition();

                // at pos0? prevent moving to the left
                if (curr == 0) {
                    ei_printf("\033[u"); // restore current position
                }
                // otherwise it's OK, move the cursor back
                else {
                    buffer.setPosition(curr - 1);

                    ei_putc('\033');
                    for (size_t ix = 0; ix < controlSequence.size(); ix++) {
                        ei_putc(controlSequence[ix]);
                    }
                }
            }
            // right
            else if (controlSequence.size() == 2 && controlSequence.at(0) == 0x5b && controlSequence.at(1) == 0x43) {
                size_t curr = buffer.getPosition();
                size_t size = buffer.size();

                // already at the end?
                if (curr == size) {
                    ei_printf("\033[u"); // restore current position
                }
                else {
                    buffer.setPosition(curr + 1);

                    ei_putc('\033');
                    for (size_t ix = 0; ix < controlSequence.size(); ix++) {
                        ei_putc(controlSequence[ix]);
                    }
                }
            }
            else {
                // not up/down? Execute original control sequence
                ei_putc('\033');
                for (size_t ix = 0; ix < controlSequence.size(); ix++) {
                    ei_putc(controlSequence[ix]);
                }
            }

            controlSequence.clear();
        }

        return;
    }

    switch (c) {
        case '\r': /* want to run the buffer */
            ei_putc(c);
            ei_putc('\n');
            runBuffer();
            break;
        case '\n': /* Ignore newline as input */
            break;
        case 0x08: /* backspace */
        case 0x7f: /* also backspace on some terminals */
            handleBackspace();
            break;
        case 0x1b: /* control character */
            // wait until next a-zA-Z
            inControlChar = true;                
            ei_printf("\033[s"); // save current position


            break; /* break out of the callback (ignore all other characters) */
        default:        	
            size_t curr_pos = buffer.getPosition();
            size_t buffer_size = buffer.size();            
            if (curr_pos == buffer_size) {
                buffer.add(c);
                ei_putc(c);
            }
            else {
                // super inefficient...
                string v(buffer.begin(), buffer.end());
                v.insert(curr_pos, 1, c);

                buffer.clear();
                buffer.add(v);

                buffer.setPosition(curr_pos + 1);

                ei_printf("\r> %s\033[%dG", v.c_str(), int(curr_pos) + 4);
            }
            break;
    }
}

static void handleBackspace(void)
{
    size_t curr_pos = buffer.getPosition();

    string v(buffer.begin(), buffer.end());

    if (v.size() == 0 || curr_pos == 0) return;

    bool endOfLine = curr_pos == v.size();

    v.erase(curr_pos - 1, 1);

    buffer.clear();
    buffer.add(v);

    buffer.setPosition(curr_pos - 1);

    if (endOfLine) {
        ei_printf("\b \b");
    }
    else {
        // carriage return, new text, set cursor, empty until end of line
        ei_printf("\r\033[K> %s\033[%dG", v.c_str(), curr_pos + 2);
    }
}

static void runBuffer(void)
{
    string rawCode(buffer.begin(), buffer.end());

    //pc->printf("Running: %s\r\n", rawCode.c_str());

    history.push_back(rawCode);
    historyPosition = history.size();

    // pc->printf("Executing (%s): ", rawCode.c_str());
    // for (size_t ix = 0; ix < rawCode.size(); ix++) {
    //     pc->printf(" %02x ", rawCode.at(ix));
    // }
    // pc->printf("\r\n");

    bool printNewState = true;
    // if (commandCallback) {
    //     printNewState = commandCallback(rawCode.c_str());
    // }  
    printNewState = ei_at_cmd_handle(rawCode.c_str());

    buffer.clear();

    if (printNewState) {
        ei_printf("> ");
    }
}
