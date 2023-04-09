/*
Définitions des caractères d'une police personnalisée avec deux tailles de caractères, de hauteur 40 et 32 pixels
Les fontes par défaut du RA8875 s'affichent très rapidement, mais elles sont peu lisibles, surtout pour le rouge, l'épaisseur des traits constituant les caractères (ou graisse) est insuffisante.
Les fontes bitmap GFX sont très esthétiques, très lisibles, mais s'affichent avec une lenteur rédhibitoire. On utilise dans ce programme la fonte GFX Arial uniquement pour les affichages constants.
Il fallait donc créer une police spéciale pour un affichage rapide et lisible. Les caractères de cette police sont inspirés des caractères des afficheurs LCD à 7 segments, d'où le nom de cette police. 
Tous les caractères sont constitués de 1 à 8 rectangles, lesquels bénéficient d'un affichage ultra rapide, accéléré au niveau hard dans le RA8875.
Comme on souhaite afficher exclusivement des nombres, la police ne comprend que 15 caractères : les 10 chiffres, le point, l'espace, le signe moins, le signe °, et la lettre C (pour Celsius)
Comme il fallait pour des raisons pratiques (pour simplifier la programmation) que les codes ASCII de ces 15 caractères se suivent, on a choisi :
  le code ASCII du '/' pour afficher le '°'
  le code ASCII du ':' pour afficher le 'C'
  le code ASCII du ';' pour afficher le ' '
 */

#ifndef FONTESLCD_H
#define FONTESLCD_H
// 1er nombre = largeur caractère, 2ème = nombre de rectangles, 3ème et 4ème = coord X et Y du 1er rectangle, 5ème et 6ème = largeur et hauteur du 1er rectangle....etc.
// Fonte lcd_32 : hauteur des caractères = 32 pixels
  const uint8_t lcd_32_0x2D[6]  = {26,1,0,13,12,6}; // caractère '-'
  const uint8_t lcd_32_0x2E[6]  = {10,1,0,26,6,6}; // caractère '.'
  const uint8_t lcd_32_0x2F[18] = {15,4,2,0,7,3,2,8,7,3,0,2,3,7,8,2,3,7}; // caractère '/' utilisé pour afficher le caratère'°'
  const uint8_t lcd_32_0x30[18] = {26,4,2,0,18,6,2,26,18,6,0,3,6,26,16,3,6,26}; // caractère '0'
  const uint8_t lcd_32_0x31[22] = {26,5,8,0,6,32,6,2,2,7,4,4,2,7,2,6,2,7,0,8,2,7}; // caractère '1'
  const uint8_t lcd_32_0x32[22] = {26,5,0,0,20,6,16,3,6,13,2,13,18,6,0,16,6,13,2,26,19,6};//caractère: '2'
  const uint8_t lcd_32_0x33[18] = {26,4,0,0,20,6,0,26,20,6,6,13,10,6,16,3,6,26};//caractère: '3' 
  const uint8_t lcd_32_0x34[34] = {26,8,13,0,6,32,0,21,22,6,11,0,2,9,9,2,2,7,7,5,2,4,5,9,6,4,3,13,6,4,1,17,6,4};//caractère: '4' 
  const uint8_t lcd_32_0x35[22] = {26,5,2,0,20,6,0,3,6,13,2,13,18,6,16,16,6,13,0,26,20,6};//caractère: '5' 
  const uint8_t lcd_32_0x36[22] = {26,5,2,0,20,6,0,3,6,26,5,13,15,6,2,26,18,6,16,16,6,13};//caractère: '6' 
  const uint8_t lcd_32_0x37[26] = {26,6,0,0,18,6,11,6,6,3,10,9,6,3,9,12,6,5,8,17,6,5,7,22,6,10};//caractère: '7'
  const uint8_t lcd_32_0x38[30] = {26,7,2,0,18,6,2,13,18,6,2,26,18,6,0,3,6,11,16,3,6,11,0,18,6,11,16,18,6,11};//caractère: '8' 
  const uint8_t lcd_32_0x39[22] = {26,5,2,0,18,6,2,13,14,6,0,26,20,6,0,3,6,14,16,3,6,26};//caractère: '9'
  const uint8_t lcd_32_0x3A[14] = {26,3,2,0,18,6,2,26,18,6,0,3,6,26};//caractère ':' utilisé pour afficher le caractère 'C' 
  const uint8_t lcd_32_0x3B[2]  = {26,0}; // caractère ';' utilisé comme ESPACE 

  const uint8_t * fontLcd32[15] = {lcd_32_0x2D, lcd_32_0x2E, lcd_32_0x2F,lcd_32_0x30, lcd_32_0x31, lcd_32_0x32, lcd_32_0x33, lcd_32_0x34, lcd_32_0x35, lcd_32_0x36, lcd_32_0x37, lcd_32_0x38, lcd_32_0x39, lcd_32_0x3A, lcd_32_0x3B };
  //  caractère :                      -             .             /            0          1             2           3            4           5              6           7            8             9            :             ;
  //  index :                          0             1             2            3          4             5           6            7           8              9           10           11            12           13            14


// Fonte lcd_40 : hauteur des caractères = 40 pixels
  const uint8_t lcd_40_0x2D[6]  = {20,1,0,16,15,8}; // caractère '-'
  const uint8_t lcd_40_0x2E[10]  = {12,2,0,33,8,6,1,32,6,8}; // caractère '.'
  const uint8_t lcd_40_0x2F[18] = {17,4,2,0,9,4,2,9,9,4,0,2,4,8,9,2,4,9}; // caractère '/' utilisé pour afficher le caratère'°'
  const uint8_t lcd_40_0x30[18] = {31,4,2,0,22,8,2,32,22,8,0,3,8,34,18,3,8,34}; // caractère '0'
  const uint8_t lcd_40_0x31[26] = {31,6,12,0,10,9,14,9,8,31,10,2,2,9,8,4,2,9,6,6,2,9,4,8,2,9}; // caractère '1'
  const uint8_t lcd_40_0x32[26] = {31,6,4,0,20,8,2,32,24,8,2,16,22,8,18,3,8,18,0,19,8,18,0,1,8,10};//caractère: '2'
  const uint8_t lcd_40_0x33[18] = {31,4,0,0,24,8,0,32,24,8,4,16,14,8,18,3,8,34};//caractère: '3' 
  const uint8_t lcd_40_0x34[34] = {31,8,14,0,8,40,0,28,26,7,11,0,3,4,9,4,5,5,7,9,6,5,5,14,7,5,3,19,7,5,1,24,7,4};//caractère: '4' 
  const uint8_t lcd_40_0x35[26] = {31,6,2,0,24,8,4,32,20,8,2,16,22,8,0,3,8,18,18,19,8,18,0,29,8,10};//caractère: '5' 
  const uint8_t lcd_40_0x36[22] = {31,5,2,0,24,8,0,3,8,34,18,19,8,18,8,16,16,8,2,32,22,8};//caractère: '6' 
  const uint8_t lcd_40_0x37[26] = {31,6,4,0,22,8,16,8,8,5,14,13,8,5,12,18,8,8,11,26,8,7,10,33,8,7};//caractère: '7'
  const uint8_t lcd_40_0x38[30] = {31,7,2,0,22,8,2,16,22,8,2,32,22,8,0,3,8,14,18,3,8,14,0,23,8,14,18,23,8,14};//caractère: '8' 
  const uint8_t lcd_40_0x39[22] = {31,5,2,0,22,8,0,3,8,18,2,16,16,8,0,32,24,8,18,3,8,34};//caractère: '9'
  const uint8_t lcd_40_0x3A[14] = {27,3,2,0,20,8,2,32,20,8,0,3,8,34};//caractère ':' utilisé pour afficher le caractère 'C' 
  const uint8_t lcd_40_0x3B[2]  = {31,0}; // caractère ';' utilisé comme ESPACE 
  
  const uint8_t * fontLcd40[15] = {lcd_40_0x2D, lcd_40_0x2E, lcd_40_0x2F,lcd_40_0x30, lcd_40_0x31, lcd_40_0x32, lcd_40_0x33, lcd_40_0x34, lcd_40_0x35, lcd_40_0x36, lcd_40_0x37, lcd_40_0x38, lcd_40_0x39, lcd_40_0x3A, lcd_40_0x3B };
  //  caractère :                      -             .             /            0          1             2           3            4           5              6           7            8             9            :             ;
  //  index :                          0             1             2            3          4             5           6            7           8              9           10           11            12           13            14
  
  
  #endif
