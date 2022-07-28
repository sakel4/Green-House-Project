#include "model.h"

Eloquent::ML::Port::SVM model;
float features[2] = { 0 };

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    // put your main code here, to run repeatedly:
    features[0] = float(51);
    features[1] = float(56);
    Serial.println("Predict Values {51,56}");
    Serial.println((model.predictLabel(features)));
    features[0] = float(95);
    features[1] = float(62);
    Serial.println("Predict Values {95,62}");
    Serial.println(model.predictLabel(features));
    features[0] = float(13);
    features[1] = float(75);
    Serial.println("Predict Values {13,75}");
    Serial.println(model.predictLabel(features));
    features[0] = float(29);
    features[1] = float(88);
    Serial.println("Predict Values {29,88}");
    Serial.println(model.predictLabel(features));
}

void loop() {
}