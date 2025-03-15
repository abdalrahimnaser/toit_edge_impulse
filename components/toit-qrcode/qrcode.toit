import system.external
import io

FUNCTION-ID ::= 0

main:
  echo := external.Client.open "toitlang.org/demo-echo"
  threshold := 0.5
  while 1:
    response := io.Reader (echo.request FUNCTION-ID "fetch result")
    fall_probability := response.little-endian.read-float32

    if fall_probability >= threshold : print "Fall Detected!"

  echo.close