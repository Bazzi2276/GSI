#define PIN_LED 7
unsigned int toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }

  Serial.println("Hello World!");
  toggle = 0;
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  for(int i = 0; i < 10; i++)
  {
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  toggle = toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
}

void loop() 
{

}

int toggle_state(int toggle)
{
  return 1 - toggle;
}
