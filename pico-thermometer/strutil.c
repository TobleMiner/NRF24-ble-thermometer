#include "strutil.h"
#include "util.h"

int long_to_str(char *str, unsigned len, long val, unsigned pfx_len) {
        char *str_start = str;
        unsigned long uval = ABS(val);
        unsigned max_radix = 0;

        do {
                uval /= 10;
                max_radix++;
        } while(uval);

        if (val < 0) {
                max_radix++;
        }

        if (max_radix + 1 > len) {
                return max_radix;
        }

        if (pfx_len + 1 > len) {
                return pfx_len;
        }

        if (val < 0) {
                *str++ = '-';
                if (max_radix) {
                        max_radix--;
                }
        }

        len = max_radix < pfx_len ? pfx_len : max_radix;

        while (pfx_len > max_radix) {
                pfx_len--;
                *str++ = '0';
        }

        uval = ABS(val);
        str += max_radix;
        *str-- = 0;

        while (max_radix--) {
                *str-- = '0' + (uval % 10);
                uval /= 10;
                if (!uval) {
                        break;
                }
        }

        return len;
}
